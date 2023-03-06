package auto;

import java.util.Objects;
import java.util.Optional;

import calc.Constants;
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
    private double startedChargePad = 0;

    // This variable is used to allow us to calibrate using the tag more often but not every loop
    private double originalNorm = 1;
    // This variable is used to tell us how far away we currently are from an april tag
    private double currentNorm = 0;

    private boolean moveArmToHumanTag = false;
  
    @Log
    private boolean coneMode = false;

    public AutoAlignment(Swerve swerve, Claw claw) {
        this.swerve = swerve;
        photonCameraPose = new PhotonCameraPose();
    }

    /**
     * Calibrate the odometry for the swerve
     */
    public void calibrateOdometry() {

      Optional<EstimatedRobotPose> result = photonCameraPose.getEstimatedRobotPose(swerve.getPose());

      // I do not believe this if statement gets what we want it to get...
      if (result.isPresent()) {

          EstimatedRobotPose camEstimatedPose = result.get();

          swerve.getPoseEstimator().addVisionMeasurement(
            camEstimatedPose.estimatedPose.toPose2d(),
            Timer.getFPGATimestamp());
            
            if (currentNorm < (originalNorm / 2) || (Objects.equals(SwerveTrajectory.trajectoryStatus, "setup") && DriverStation.isTeleop())) {
              
              // swerve.getPoseEstimator().addVisionMeasurement(
              //   camEstimatedPose.estimatedPose.toPose2d(),
              //   Timer.getFPGATimestamp());

              setTagID(getNearestTag());

              if (photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
                originalNorm = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d()).getTranslation().getNorm();

                if (!Objects.equals(SwerveTrajectory.trajectoryStatus, "setup") && (tagID == 4 || tagID == 5)) {

                  moveArmToHumanTag = true;
                  
                  double normToLeftOfHumanTag = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().plus(new Transform2d(
                      new Translation2d(
                          (AlignmentConstants.GRID_BARRIER_METERS),
                          AlignmentConstants.CONE_OFFSET_METERS),
                      new Rotation2d()))).getTranslation().getNorm();

                  double normToRightOfHumanTag = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().plus(new Transform2d(
                      new Translation2d(
                          (AlignmentConstants.GRID_BARRIER_METERS),
                          -AlignmentConstants.CONE_OFFSET_METERS),
                      new Rotation2d()))).getTranslation().getNorm();

                  /*
                    If we are on red alliance, left of the tag is going to be the negative on the Y axis
                    Due to moveToTag checking if we are on the blue alliance and flipping the sign,
                    we need to flip the sign here
                  */
                  if (normToLeftOfHumanTag < normToRightOfHumanTag) {
                    this.substationOffset = 1;
                    // Start intaking the claw when we get close to the tag
                    if (normToLeftOfHumanTag < 2) {
                      claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED);
                    }
                  } else {
                    this.substationOffset = -1;
                    // Start intaking the claw when we get close to the tag
                    if (normToRightOfHumanTag < 2) {
                      claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED);
                    }
                  }

                } else {
                  moveArmToHumanTag = false;
                }
              }

            }
          // System.out.println(currentNorm + " " + originalNorm);
      }
    }

    public void moveToTag() {

      // If we cannot see a tag
      if (tagID == 0) {
          return;
      }

      Pose2d targetPose = photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();

      currentNorm = swerve.getPose().minus(targetPose).getTranslation().getNorm();

      // If we are on the left side of the field, we need to add the grid offset + cone/substation offset
      // If we are  on the right side of the field, we need to subtract the grid offset + cone/substation offset
      // If we are going to a substation, we need to add the substation offset instead of the cone offset
      // We add the grid length to both because we still want to be a small bit away from the tag
      if (0 < tagID && tagID < 5) {
          targetPose = targetPose.plus(new Transform2d(
              new Translation2d(
                  (tagID == 4) ?
                      -(PlacementConstants.HUMAN_TAG_PICKUP.getX() + Constants.ClawConstants.CLAW_LENGTH_INCHES) :
                      (AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH/2) + PlacementConstants.BUMPER_LENGTH),
                  (tagID == 4) ?
                      (AlignmentConstants.SUBSTATION_OFFSET_METERS * this.substationOffset) :
                      -(AlignmentConstants.CONE_OFFSET_METERS * this.coneOffset)),
              Rotation2d.fromDegrees(180)));
      } else {
          targetPose = targetPose.plus(new Transform2d(
              new Translation2d(
                  (tagID == 5) ?
                      (PlacementConstants.HUMAN_TAG_PICKUP.getX() - Constants.ClawConstants.CLAW_LENGTH_INCHES) :
                      -(AlignmentConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH/2) + PlacementConstants.BUMPER_LENGTH),
                  ((tagID == 5) ?
                      (AlignmentConstants.SUBSTATION_OFFSET_METERS * this.substationOffset) :
                      -(AlignmentConstants.CONE_OFFSET_METERS * this.coneOffset))),
              Rotation2d.fromDegrees(180)));
      }

      // If we are close enough to the tag, stop moving
      if (currentNorm < AlignmentConstants.ALLOWABLE_ERROR_METERS) {
          swerve.drive(0, 0, 0, false);
          SwerveTrajectory.trajectoryStatus = "done";
          return;
      }

      // Calculate the direct heading to our destination, so we can drive straight to it
      Rotation2d heading = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - swerve.getPose().getY(),targetPose.getX() - swerve.getPose().getX()));

      PathPlannerTrajectory tagTrajectory = PathPlanner.generatePath
      (
          new PathConstraints(DriveConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
          new PathPoint(swerve.getPose().getTranslation(),
              heading,
              swerve.getPose().getRotation(), 
              // start the path at the current speed of the robot
              swerve.getSpeedMetersPerSecond()),

          new PathPoint(targetPose.getTranslation(),
              heading,
              targetPose.getRotation())
      );

      SwerveTrajectory.PathPlannerRunner(tagTrajectory, swerve);

      // System.out.println("April Pose: " + photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d());
      // System.out.println("Modified Target Pose: " + targetPose);
      // System.out.println("Current Pose: " + swerve.getPose() + "\n\n");
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
      double currentDistance;

      Translation2d currentPosition = swerve.getPose().getTranslation();

      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        for (int i = 8; i > 4; i--) {
          // Tag 4 is for the red alliance
          if (i == 5) { i = 4; }

          currentDistance = currentPosition.getDistance(photonCameraPose.aprilTagFieldLayout.getTagPose(i).get().toPose2d().getTranslation());

          if (currentDistance < nearestDistance) {

            nearestDistance = currentDistance;
            nearestTag = i;

          }
        }
      }
      else {
        for (int i = 1; i < 5; i++) {
          // Tag 4 is for the blue alliance
          if (i == 4) { i = 5; }

          currentDistance = currentPosition.getDistance(photonCameraPose.aprilTagFieldLayout.getTagPose(i).get().toPose2d().getTranslation());

          if (currentDistance < nearestDistance) {

            nearestDistance = currentDistance;
            nearestTag = i;

          }
        }
      }
      return nearestTag;
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
          
          System.out.println("Case 1: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
        else if (tagID == 6 || tagID == 7) {
          this.tagID++;
          coneOffset = (coneMode) ? 1 : 0; 
          
          System.out.println("Case 2: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
      }
      else if (coneOffset > 1) {
        if (tagID == 1 || tagID == 2) {
          this.tagID++;
          coneOffset = (coneMode) ? -1 : 0; 
          
          System.out.println("Case 3: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
        else if (tagID == 7 || tagID == 8) {
          this.tagID--;
          coneOffset = (coneMode) ? -1 : 0; 
          
          System.out.println("Case 4: Tag = " + tagID + ", coneOffset = " + coneOffset);
        }
      }
      System.out.println(this.coneOffset + " to " + coneOffset);

      this.coneOffset = MathUtil.clamp(coneOffset, -1, 1);

      if (!coneMode) {
        this.coneOffset = 0;
      }

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
      this.coneMode = coneMode;
      // Set the current cone offset to the left of a tag if it is zero
      this.coneOffset = ((this.coneOffset == 0 && coneMode) ? ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? -1 : 1) : 0);
    }

    public boolean getConeMode() {
      return this.coneMode;
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

      // System.out.println(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/16)));

      if (tilt > Math.toRadians(7)) {
        swerve.drive(
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/16)), 0.05, 0.20),
            0, 
            0, 
            true);
      }
      else if (tilt < -Math.toRadians(7)) {
        swerve.drive(
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * tilt)/(elapsedTime/16)), -0.20, -0.05),
            0, 
            0, 
            true);
      }
      else {
        swerve.setWheelsUp();
      }
    }

    public void startChargePad() {
      startedChargePad = Timer.getFPGATimestamp();
    }
  
}