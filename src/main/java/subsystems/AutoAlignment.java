package subsystems;

import auto.SwerveTrajectory;

import java.util.Objects;
import java.util.Optional;

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
import math.Constants.AlignmentConstants;
import math.Constants.DriveConstants;
import math.Constants.PlacementConstants;

public class AutoAlignment {

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
    PhotonCameraPose photonCameraPose;

    private int tagID;
    private int coneOffset;
    private double startedChargePad = 0;

    // This variable is used to allow us to calibrate using the tag more often but not every loop
    private double originalNorm = 1;
    // This variable is used to tell us how far away we currently are from an april tag
    private double currentNorm = 0;

    private boolean moveArmToHumanTag = false;

    public AutoAlignment(Swerve swerve) {
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

            
            
            if (currentNorm < (originalNorm / 2) || Objects.equals(SwerveTrajectory.trajectoryStatus, "setup")) {
              
              swerve.getPoseEstimator().addVisionMeasurement(
                camEstimatedPose.estimatedPose.toPose2d(),
                Timer.getFPGATimestamp());
                                
              setTagID(getNearestTag());

              if (photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
                originalNorm = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d()).getTranslation().getNorm();

                if (!Objects.equals(SwerveTrajectory.trajectoryStatus, "setup") && (tagID == 4 || tagID == 5)) {

                  moveArmToHumanTag = true;
                  
                  double normToLeftOfHumanTag = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().plus(new Transform2d(
                      new Translation2d(
                          (AlignmentConstants.GRID_BARRIER),
                          AlignmentConstants.CONE_OFFSET_METERS),
                      new Rotation2d()))).getTranslation().getNorm();

                  double normToRightOfHumanTag = swerve.getPose().minus(photonCameraPose.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().plus(new Transform2d(
                      new Translation2d(
                          (AlignmentConstants.GRID_BARRIER),
                          -AlignmentConstants.CONE_OFFSET_METERS),
                      new Rotation2d()))).getTranslation().getNorm();

                  /*
                    If we are on red alliance, left of the tag is going to be the negative on the Y axis
                    Due to moveToTag checking if we are on the blue alliance and flipping the sign,
                    we need to flip the sign here
                  */
                  if (normToLeftOfHumanTag < normToRightOfHumanTag) {
                    coneOffset = 1;
                  } else {
                    coneOffset = -1;
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
      
      double coneOffsetLeft = AlignmentConstants.CONE_OFFSET_METERS * coneOffset;

      if (0 < tagID && tagID < 5) {
          targetPose = targetPose.plus(new Transform2d(
              new Translation2d(
                  (AlignmentConstants.GRID_BARRIER + (PlacementConstants.ROBOT_LENGTH/2) + PlacementConstants.BUMPER_LENGTH),
                  0),
              Rotation2d.fromDegrees(180)));
      } else {
          targetPose = targetPose.plus(new Transform2d(
              new Translation2d(
                  -(AlignmentConstants.GRID_BARRIER + (PlacementConstants.ROBOT_LENGTH/2) + PlacementConstants.BUMPER_LENGTH),
                  0),
              Rotation2d.fromDegrees(180)));
      }

      targetPose = targetPose.plus(new Transform2d(new Translation2d(0, coneOffsetLeft), Rotation2d.fromDegrees(0)));

      if (currentNorm < AlignmentConstants.ALLOWABLE_ERROR) {
          swerve.drive(0, 0, 0, false);
          SwerveTrajectory.trajectoryStatus = "done";
          return;
      }

      Rotation2d heading = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - swerve.getPose().getY(),targetPose.getX() - swerve.getPose().getX() ));

      PathPlannerTrajectory tagTrajectory = PathPlanner.generatePath
      (
          new PathConstraints(DriveConstants.MAX_SPEED_METERS_PER_SECOND, 2.5),
          new PathPoint(swerve.getPose().getTranslation(),
              heading,
              swerve.getPose().getRotation()),
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
      double currentDistance = 0;

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

      // Pan the coneOffset to the next tag if it is able to do so
      // It cannot do so if there is no grid in the desired direction
      if (coneOffset < -1) {
        if (tagID == 2 || tagID == 3) {
          this.tagID--;
          coneOffset = 1;
          System.out.println("Tag--, coneOffset = 1");
        }
        else if (tagID == 6 || tagID == 7) {
          this.tagID++;
          coneOffset = 1;
          System.out.println("Tag++, coneOffset = 1");
        }
      }
      else if (coneOffset > 1) {
        if (tagID == 1 || tagID == 2) {
          this.tagID++;
          coneOffset = -1;
          System.out.println("Tag++, coneOffset = -1");
        }
        else if (tagID == 7 || tagID == 8) {
          this.tagID--;
          coneOffset = -1;
          System.out.println("Tag--, coneOffset = -1");
        }
      }
      System.out.println(this.coneOffset + " to " + coneOffset);

      this.coneOffset = MathUtil.clamp(coneOffset, -1, 1);
      
      if (previousConeOffset != this.coneOffset) {
        SwerveTrajectory.resetTrajectoryStatus();
      }

      
    }

    public boolean getMoveArmToHumanTag() {
      return this.moveArmToHumanTag;
    }

    public void chargeAlign() {

      double elapsedTime = Timer.getFPGATimestamp() - startedChargePad;
      // boolean setWheelsUp = false;
      double roll = swerve.roll + ((swerve.roll < 0) ? 180 : -180);

      if (roll > 7) {
        swerve.drive(-
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * roll)/(elapsedTime * ((DriverStation.isAutonomous()) ? 3 : 1.5))), -0.5, 0.5), 
            0, 
            0, 
            true);
      }
      else if (roll < -7) {
        swerve.drive(-
            MathUtil.clamp(((AlignmentConstants.CHARGE_PAD_CORRECTION_P * roll)/(elapsedTime * ((DriverStation.isAutonomous()) ? 3 : 1.5))), -0.5, 0.5), 
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