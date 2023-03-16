package auto;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint; 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import hardware.Swerve;
import hardware.Claw;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import calc.Constants.AlignmentConstants;
import calc.Constants.AutoConstants;
import calc.Constants.DriveConstants;
import calc.Constants.PlacementConstants;
import calc.Constants.ClawConstants;
import calc.PhotonCameraPose;

public class AutoAlignment implements Loggable{

    /**
     * A visual representation of the apriltag positions
     * / --------------------------------------------- \
     * 5                     |                       4
     * |                      |                        |
     * |                      |                        |
     * 6                      |                        3
     * |                      |                        |
     * 7                      |                        2
     * |                      |                        |
     * 8                      |                        1
     * \ --------------------------------------------- /
     */

    Swerve swerve;
    Claw claw;
    PhotonCameraPose photonCameraPose;

    private int tagID;
    private int coneOffset = 0;
    private int substationOffset = -1;
    private double startedChargePad = -1;

    // This variable is used to allow us to calibrate using the tag more often, but not every loop
    private double originalNorm = 1;
    // This variable is used to tell us how far away we currently are from an april tag
    private double currentNorm = -1;

    private boolean moveArmToHumanTag = false;
  
    @Log
    public static boolean coneMode = false;

    public AutoAlignment(Swerve swerve, Claw claw) {
        this.swerve = swerve;
        this.claw = claw;
        photonCameraPose = new PhotonCameraPose();
    }

    /**
     * Calibrate the odometry for the swerve
     */
    public void calibrateOdometry() {

      // Create an "Optional" object that contains the estimated pose of the robot
      // This can be present (see's tag) or not present (does not see tag)
      Optional<EstimatedRobotPose> result = photonCameraPose.getEstimatedRobotPose(swerve.getPose());

      // I do not believe this if a statement gets what we want it to get...
      if (result.isPresent()) {

        EstimatedRobotPose camEstimatedPose = result.get();

        // Add the vision measurement to the pose estimator to update the odometry
        swerve.getPoseEstimator().addVisionMeasurement(
          camEstimatedPose.estimatedPose.toPose2d(),
          Timer.getFPGATimestamp());

          // If we are half the distance from the last "originalNorm" we were at, reset originalNorm
          // This is primarily used to tell the arm to move halfway through the path
          if (currentNorm < (originalNorm / 2) || (Objects.equals(SwerveTrajectory.trajectoryStatus, "setup") && DriverStation.isTeleop())) {

            if (photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
              originalNorm = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d()).getTranslation().getNorm();

              if (!Objects.equals(SwerveTrajectory.trajectoryStatus, "setup") && (tagID == 4 || tagID == 5)) {

                moveArmToHumanTag = true;

                // Find which side of the human tag we are closest to based on the tag ID's location and the robot's location
                double negativeOffsetNorm = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().plus(new Transform2d(
                    new Translation2d(
                        // We know that the tag is going to be either 4 or 5, due to the if statement above ^^
                        (tagID == 4) ?
                            // Tag 4 means we need to subtract the grid barrier
                            // since it is on the right side of the field
                            // (red alliance)
                            -(AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS/2) + PlacementConstants.BUMPER_LENGTH_METERS) :
                            // Tag 5 means we need to add the grid barrier
                            (AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS/2) + PlacementConstants.BUMPER_LENGTH_METERS),
                        -AlignmentConstants.CONE_OFFSET_METERS),
                    new Rotation2d()))).getTranslation().getNorm();

                double positiveOffsetNorm = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().plus(new Transform2d(
                    new Translation2d(
                        // See comment 9ish lines above ^^
                        (tagID == 4) ?
                            // Tag 4 means we need to subtract the grid barrier
                            // since it is on the right side of the field
                            // (red alliance)
                            -(AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS/2) + PlacementConstants.BUMPER_LENGTH_METERS) :
                            // Tag 5 means we need to add the grid barrier
                            (AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS/2) + PlacementConstants.BUMPER_LENGTH_METERS),
                        AlignmentConstants.CONE_OFFSET_METERS),
                    new Rotation2d()))).getTranslation().getNorm();

                /*
                  If we are on red alliance, left of the tag is going to be the negative on the Y axis
                  Due to this.mveToTag() checking if we are on the blue alliance and flipping the sign,
                  we need to flip the sign here
                */
                if (negativeOffsetNorm < positiveOffsetNorm) {
                  this.substationOffset = -1;
                  // Start intaking the claw when we get close to the tag
                  if (negativeOffsetNorm < 2) {
                    claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CONE);
                  }
                } else {
                  this.substationOffset = 1;
                  // Start intaking the claw when we get close to the tag
                  if (positiveOffsetNorm < 2) {
                    claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CONE);
                  }
                }

              } else {
                moveArmToHumanTag = false;
              }
            }
          }
          // System.out.println(currentNorm + " " + originalNorm);
          
          if (photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
            // Get the target pose (the pose of the tag we want to go to)
            Pose2d targetPose = photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
            targetPose = getModifiedTargetPose(targetPose);
            currentNorm = swerve.getPose().minus(targetPose).getTranslation().getNorm();
        }
      }
    }

    public void moveToTag() {

      // If we cannot see a tag
      if (tagID == 0) {
        return;
      }
      Pose2d targetPose = swerve.getPose();

      // Check if our tagID is valid... (Assume it is for logic purposes)
      if (photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
          // Get the target pose (the pose of the tag we want to go to)
          targetPose = photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
      }

      // If we are on the left side of the field: we need to add the grid offset + cone/substation offset
      // If we are on the right side of the field: we need to subtract the grid offset + cone/substation offset
      // If we are going to a substation: we need to add the substation offset instead of the cone offset
      // We add the grid length to both because we still want to be a small bit away from the tag
      // There is a bit of a logic issue that the else statement "should" be subtracting, but it doesn't work when you do that...
      // oh well.
      targetPose = getModifiedTargetPose(targetPose);

      currentNorm = swerve.getPose().minus(targetPose).getTranslation().getNorm();

      // Calculate the direct heading to our destination, so we can drive straight to it
      Rotation2d heading = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - swerve.getPose().getY(),targetPose.getX() - swerve.getPose().getX()));

      PathPlannerTrajectory tagTrajectory = PathPlanner.generatePath
      (
          new PathConstraints(DriveConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED/3),
          new PathPoint(swerve.getPose().getTranslation(),
              heading,
              swerve.getPose().getRotation(),
              swerve.getSpeedMetersPerSecond()),

          new PathPoint(targetPose.getTranslation(),
              heading,
              targetPose.getRotation(), 0)
      );
      
      // System.out.println("April Pose: " + photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d());
      // System.out.println("Modified Target Pose: " + targetPose);
      // System.out.println("Current Pose: " + swerve.getPose() + "\n\n");
      
      SwerveTrajectory.PathPlannerRunner(tagTrajectory, swerve);
    }


    /**
     * Get the tag nearest to the robot using its position
     * while using the alliance color to factor out tags
     * @return the nearest tag to the bot that is for the same alliance
     */
    public int getNearestTag() {

      // A reminder that tag 0 sets this.moveToTag() to return;
      int nearestTag = 0;
      double nearestDistance = 1000;
      double currentDistance = 1000;

      Translation2d currentPosition = swerve.getPose().getTranslation();

      // If we are on the blue alliance, only look at positions of tags 8,7,6 and 4
      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        for (int i = 8; i > 4; i--) {
          // Tag 5 is for the red alliance's substation, not ours
          // Skip over to our substation
          if (i == 5) { i = 4; }

          // This if a statement prevents the robot from crashing if we input an absurd tag ID,
          // but it should be assumed that the tag location is present.
          if (photonCameraPose.aprilTagFieldLayout.getTagPose(i).isPresent()) {
            currentDistance = currentPosition.getDistance(photonCameraPose.aprilTagFieldLayout.getTagPose(i).get().toPose2d().getTranslation());
          }
          if (currentDistance < nearestDistance) {
            nearestDistance = currentDistance;
            nearestTag = i;
          }
        }
      }
      // We are on the red alliance, only look at positions of tags 1,2,3 and 5
      else {
        for (int i = 1; i < 5; i++) {
          // Tag 4 is for the blue alliance's substation, not ours
          // Skip over to our substation
          if (i == 4) { i = 5; }

          // This if a statement prevents the robot from crashing if we input an absurd tag ID,
          // but it should be assumed that the tag location is present.
          if (photonCameraPose.aprilTagFieldLayout.getTagPose(i).isPresent()) {
            currentDistance = currentPosition.getDistance(photonCameraPose.aprilTagFieldLayout.getTagPose(i).get().toPose2d().getTranslation());
          }
          if (currentDistance < nearestDistance) {
            nearestDistance = currentDistance;
            nearestTag = i;
          }
        }
      }

      System.out.println("Current nearest tag " + nearestTag + " at distance " + Units.metersToInches(nearestDistance) + " Inches");

      return nearestTag;
    }

    /**
     * Get the modified target pose based on the alliance color
     * @param targetPose the target pose of the tag we want to go to
     * @return the modified target pose using constants for grid/substation
     */
    private Pose2d getModifiedTargetPose(Pose2d targetPose) {
    if (0 < tagID && tagID < 5) {

      targetPose = targetPose.plus(new Transform2d(
          new Translation2d(
              (tagID == 4) ?
                  Units.inchesToMeters(PlacementConstants.HUMAN_TAG_PICKUP.getX() + ClawConstants.CLAW_LENGTH_INCHES) :
                  (AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS/2) + PlacementConstants.BUMPER_LENGTH_METERS),
              (tagID == 4) ?
                  -(AlignmentConstants.SUBSTATION_OFFSET_METERS * this.substationOffset) :
                  -(AlignmentConstants.CONE_OFFSET_METERS * this.coneOffset)),
          Rotation2d.fromDegrees(180)));

    } else {

      targetPose = targetPose.plus(new Transform2d(
          new Translation2d(
              (tagID == 5) ?
                  Units.inchesToMeters(PlacementConstants.HUMAN_TAG_PICKUP.getX() - ClawConstants.CLAW_LENGTH_INCHES) :
                  (AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS/2) + PlacementConstants.BUMPER_LENGTH_METERS),
              ((tagID == 5) ?
                  (AlignmentConstants.SUBSTATION_OFFSET_METERS * this.substationOffset) :
                  (AlignmentConstants.CONE_OFFSET_METERS * this.coneOffset))),
          Rotation2d.fromDegrees(180)));

    }
    return targetPose;
  }

    public int getTagID() {
        return tagID;
    }

    public void setTagID(int tagID) {
        this.tagID = tagID;
    }

    public int getConeOffset() {
        return this.coneOffset;
    }

    public void setConeOffset(int coneOffset) {
      
      int previousConeOffset = this.coneOffset;

      // If we are on cone mode, skip cone offsets of 0
      // also, skip straight to the next cone if we are on the same cone
      // in other words,
      // if we are on cone mode, at cone index 1, and we want to go to cone index 2, skip cone index 0
      // or,
      // if we are *not* on cone mode, (at cone index 0) and we want to go to cone index 1, skip cone index 1 and go to 2.
      // This is because the call from robot.java will run as (setConeOffset(getConeOffset +/- 1))
      if (coneMode) {
        if (previousConeOffset == -1) {
          if (coneOffset == 0) {
            coneOffset = 1;
          }
        }
        else if (previousConeOffset == 1) {
          if (coneOffset == 0) {
            coneOffset = -1;
          }
        }
      }
      else {
        this.coneOffset = 0;
        if (coneOffset == -1) {
          coneOffset = -2;
        }
        else if (coneOffset == 1) {
          coneOffset = 2;
        }
      }

      // Pan the coneOffset to the next tag if it is able to do so
      // It cannot do so if there is no grid in the desired direction
      if (coneOffset < -1) {
        if (tagID == 2 || tagID == 3) {
          this.tagID--;
          coneOffset = (coneMode) ? 1 : 0;
          // System.out.println("Case 1: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
        else if (tagID == 6 || tagID == 7) {
          this.tagID++;
          coneOffset = (coneMode) ? 1 : 0;
          // System.out.println("Case 2: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
      }
      else if (coneOffset > 1) {
        if (tagID == 1 || tagID == 2) {
          this.tagID++;
          coneOffset = (coneMode) ? -1 : 0;
          // System.out.println("Case 3: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
        else if (tagID == 7 || tagID == 8) {
          this.tagID--;
          coneOffset = (coneMode) ? -1 : 0;
          // System.out.println("Case 4: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
      }

      System.out.println(this.coneOffset + " to " + coneOffset);

      // Clamp the cone offset to -1, 0, or 1
      this.coneOffset = MathUtil.clamp(coneOffset, -1, 1);

      // If we are not on cone mode, ensure the cone offset is zero
      if (!coneMode) {
        this.coneOffset = 0;
      }

      // If we actually changed indexes, reset the auto alignment status,
      // so we can re-align to the new index
      if (previousConeOffset != this.coneOffset) {
        SwerveTrajectory.resetTrajectoryStatus();
      }
    }

    public boolean getMoveArmToHumanTag() {
      return this.moveArmToHumanTag;
    }

    public int getSubstationOffset() {
      return this.substationOffset;
    }

    public void setSubstationOffset(int substationOffset) {
      this.substationOffset = MathUtil.clamp(substationOffset, -1, 1);
    }

    public void setConeMode(boolean coneMode) {
      AutoAlignment.coneMode = coneMode;

      // If the cone offset is 0, and we are switching to cone mode,
      // set the cone offset to 1 (closest to human tag)
      if (coneMode && this.coneOffset == 0) {
        this.coneOffset = 1;
      }
      else if (!coneMode) {
        this.coneOffset = 0;
      }
    }

    public double getCurrentNorm() {
      return this.currentNorm;
    }

    public void chargeAlign() {

      double elapsedTime = Timer.getFPGATimestamp() - startedChargePad;
      // boolean setWheelsUp = false;
      double tilt = 0;

      // If our heading is within -45 to 45 degrees or within -135 and -180 or within 135 to 180, use the pitch
      // Otherwise, use the roll
      if (-45 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < 45) {
        tilt = -swerve.getPitch().getRadians();
      }
      else if (-180 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < -135 ||
          135 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < 180) 
      {
        tilt = swerve.getPitch().getRadians();
      }
      else if (-135 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < -45) {
        tilt = swerve.getRoll().getRadians();
      }
      else if (45 < swerve.getYaw().getDegrees() && swerve.getYaw().getDegrees() < 135) 
      {
        tilt = -swerve.getRoll().getRadians();
      }

      System.out.printf("Elapsed Time: %.1f, Full output: %.2f\n", elapsedTime, ((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/16)));

      if (tilt > Math.toRadians(7)) {
        swerve.drive(
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/24)), 0.055, 0.20),
            0, 
            0, 
            true, false);
      }
      else if (tilt < -Math.toRadians(7)) {
        swerve.drive(
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/24)), -0.20, -0.055),
            0, 
            0, 
            true, false);
      }
      else {
        swerve.setWheelsUp();
      }
    }

    public void startChargePad() {
      startedChargePad = Timer.getFPGATimestamp();
    }

    public void snapToAngle(Translation2d driverAxis, Rotation2d desiredAngle) {

      // Make a simple trajectory for use in the Holonomic Drive Controller
      // Notice that the start and end states have the same position, but different holonomic rotations
      PathPlannerTrajectory rotationalTrajectory = PathPlanner.generatePath
          (
              new PathConstraints(DriveConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED/3),
              new PathPoint(swerve.getPose().getTranslation(),
                  Rotation2d.fromDegrees(0),
                  swerve.getYaw()),

              new PathPoint(swerve.getPose().getTranslation(),
                  Rotation2d.fromDegrees(0),
                  desiredAngle)
          );

      // Use a Holonomic Drive Controller to calculate the speeds for the robot
      var trajectorySpeeds = SwerveTrajectory.HDC.calculate(swerve.getPose(), rotationalTrajectory.getInitialState(), desiredAngle);
      // Notice that only the turning speed is used. We still want to be able to drive forward and strafe
      // One strange thing that I noticed is that the HDC generally doesn't use field relative to drive,
      // I wonder if that will cause issues in the future.
      swerve.drive(driverAxis.getY(), driverAxis.getX(), trajectorySpeeds.omegaRadiansPerSecond, true, true);
    }
}