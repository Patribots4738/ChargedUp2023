package subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory.State;
import auto.*;
import hardware.Swerve;
import math.Constants;
import math.Constants.AlignmentConstants;
import math.Constants.VisionConstants;


public class AutoAlignment {

  /**
   *  A visual representation of the apriltag positions
   *  / --------------------------------------------- \ 
   *   5                     |                       4
   *  |                      |                        |
   *  |                      |                        |
   *  6                      |                        3
   *  |                      |                        |
   *  7                      |                        2
   *  |                      |                        |
   *  8                      |                        1
   *  \ --------------------------------------------- /
   */
  
  Swerve swerve;

  SwerveDriveOdometry odometry;

  Pose2d targetPose;

  int tagID;

  public AutoAlignment(Swerve swerve) {
    this.swerve = swerve;
  }

  /**
   * Calibrate the odometry for the swerve
   * @param visionPitch the position of the aprilTag relative to the bot
   * @param aprilTagID the ID of the tag being watched
   */
  public void calibrateOdometry(int aprilTagID, Transform3d visionTransform3d) {

    double headingToReference = visionTransform3d.getRotation().getZ() + Math.PI * -Math.signum(visionTransform3d.getRotation().getZ());

    Translation2d targetPosition = getTagPos(aprilTagID).getTranslation();
    Translation2d visionTranslation2d = visionTransform3d.getTranslation().toTranslation2d().unaryMinus();

    Translation2d robotPose = targetPosition.plus(visionTranslation2d).plus(VisionConstants.CAMERA_POSITION.getTranslation().toTranslation2d());

    swerve.resetOdometry(new Pose2d(robotPose, Rotation2d.fromRadians(headingToReference)));

    SwerveTrajectory.resetTrajectoryStatus();

  }

  public boolean isAtTarget(Pose2d targetPose) {
    boolean atTarget = swerve.getOdometry().getPoseMeters().getTranslation().getX() 
                      - targetPose.getTranslation().getX() < 0.1; // close enough to target
    return atTarget;
  }

  public Pose2d moveToTag(int tagID, HolonomicDriveController HDC, AutoSegmentedWaypoints autoSegmentedWaypoints, int coneOffset) {

    // autoSegmentedWaypoints.periodic();

    Pose2d targetPose = getTagPos(tagID);
    
    if (0 < tagID && tagID < 5) {
        targetPose = targetPose.plus(new Transform2d(new Translation2d(AlignmentConstants.GRID_BARRIER, 0), Rotation2d.fromDegrees(180)));
    } else {
        targetPose = targetPose.plus(new Transform2d(new Translation2d(-AlignmentConstants.GRID_BARRIER, 0), Rotation2d.fromDegrees(0)));
    }
    if (coneOffset == 1) {
      targetPose = targetPose.plus(new Transform2d(new Translation2d(0, VisionConstants.CONE_OFFSET_METERS), Rotation2d.fromDegrees(0)));
    }
    else if (coneOffset == -1) {
      targetPose = targetPose.plus(new Transform2d(new Translation2d(0, -VisionConstants.CONE_OFFSET_METERS), Rotation2d.fromDegrees(0)));
    }
    
    if (isAtTarget(targetPose)) {
      return targetPose;
    }

    PathPlannerTrajectory tagPos = PathPlanner.generatePath(
      new PathConstraints(0.1, 0.1),
      new PathPoint(
        swerve.getOdometry().getPoseMeters().getTranslation(),
        // Might be useful to make this heading dynamic based on tag...
        // @see pathplanner
        Rotation2d.fromDegrees(0), 
        swerve.getOdometry().getPoseMeters().getRotation()
      ),
      new PathPoint(
        targetPose.getTranslation(),
        Rotation2d.fromDegrees(0),
        targetPose.getRotation()
      )
    
      );
    
    SwerveTrajectory.PathPlannerRunner(tagPos, swerve, swerve.getOdometry(), swerve.getOdometry().getPoseMeters().getRotation());
    
    System.out.println("April Pose: " + getTagPos(tagID));
    System.out.println("Modified Target Pose: " + targetPose);
    System.out.println("Current Pose: " + swerve.getOdometry().getPoseMeters() + "\n\n");

    return null;
  }

  public void moveRelative(double x, double y, double rotation, HolonomicDriveController HDC) {

    Pose2d currentPose = swerve.getOdometry().getPoseMeters();
    
    Pose2d targetPose = new Pose2d(
      currentPose.getX() + x,
      currentPose.getY() + y,
      new Rotation2d(currentPose.getY() + rotation));

    State targetState = new State(0, 0, 0, targetPose, 0);

    ChassisSpeeds speeds = HDC.calculate(
      currentPose,
      targetState,
      targetPose.getRotation());

    swerve.drive(speeds.vxMetersPerSecond*0.25,
      speeds.vyMetersPerSecond*0.25, 
      speeds.omegaRadiansPerSecond,false);

  }

  public boolean isAligned() {
    if (swerve.getOdometry().getPoseMeters() == targetPose) {
      return true;
    }
    return false;
  }

  private Pose2d getTagPos(int tagID) {
    double tagX = 0.0;
    double tagY = 0.0;
    double tagZ = 0.0;
    Rotation2d rotation = Rotation2d.fromRadians(0);

    switch (tagID) {
      case 1:
        tagX = Constants.AlignmentConstants.TAG_1_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_1_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_1_POSE.getZ();
        rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
        break;

      case 2:
        tagX = Constants.AlignmentConstants.TAG_2_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_2_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_2_POSE.getZ();
        rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
        break;

      case 3:
        tagX = Constants.AlignmentConstants.TAG_3_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_3_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_3_POSE.getZ();
        rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
        break;

      case 6:
        tagX = Constants.AlignmentConstants.TAG_7_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_7_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_7_POSE.getZ();
        rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
        break;

      case 7:
        tagX = Constants.AlignmentConstants.TAG_8_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_8_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_8_POSE.getZ();
        rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
        break;

      case 8: 
        tagX = Constants.AlignmentConstants.TAG_8_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_8_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_8_POSE.getZ();
        rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
        break;
    }
    return new Pose2d(tagX, tagY, rotation);
  }

  public void setTagID(int tagID) {
    this.tagID = tagID;
  }

  public int getTagID() {
    return tagID;
  }

}