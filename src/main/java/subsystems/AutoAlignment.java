package subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import javax.sql.rowset.spi.TransactionalWriter;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory.State;
import auto.*;
import hardware.Swerve;
import math.Constants;
import math.Constants.VisionConstants;


public class AutoAlignment {

  /**
   *  A visual representation of the apriltag positions
   *  / --------------------------------------------- \ 
   *  5                      |                        4
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
  public void calibrateOdometry(int aprilTagID, double visionPitch, double visionYaw, Transform3d visionPose) {
    
    double rangeMeters =
                  PhotonUtils.calculateDistanceToTargetMeters(
                          VisionConstants.kCameraPosition.getZ(),
                          getTagPos(aprilTagID).getZ(),
                          VisionConstants.kCameraPosition.getRotation().getY(),
                          Units.degreesToRadians(visionPitch));

    Translation2d targetPosition = getTagPos(aprilTagID).toPose2d().getTranslation();

    Pose2d generatedPose = new Pose2d(
      PhotonUtils.estimateCameraToTargetTranslation(rangeMeters, Rotation2d.fromDegrees(-visionYaw)).plus(targetPosition),
      Rotation2d.fromDegrees(visionYaw)
    );

    // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(visionPose, getTagPos(aprilTagID), VisionConstants.kCameraPosition);
    
    swerve.resetOdometry(generatedPose);

    SwerveTrajectory.resetTrajectoryStatus();

  }

  public void moveToTag(int tagID, HolonomicDriveController HDC, AutoWaypoints autoWaypoints) {

    autoWaypoints.autoPeriodic();

    Pose2d targetPose = getTagPos(tagID).toPose2d();

    
    // if (0 < tagID && tagID < 4) {
      //   targetPose = targetPose.plus(new Transform2d(new Translation2d(-Units.inchesToMeters(15), 0), new Rotation2d(0)));
      // } else {
        //   targetPose = targetPose.plus(new Transform2d(new Translation2d(Units.inchesToMeters(15), 0), new Rotation2d(0)));
        // }
        
    int direction = (0 < tagID && tagID < 5) ? -1 : 1;

    targetPose = new Pose2d(
      new Translation2d(
        targetPose.getX() + ((Units.inchesToMeters(15) + (VisionConstants.kCameraPosition.getX())) * direction), 
        targetPose.getY() + (VisionConstants.kCameraPosition.getY())
      ),
      targetPose.getRotation()
    );

    PathPlannerTrajectory tagPos = PathPlanner.generatePath(
      new PathConstraints(0.1, 0.1),
      new PathPoint(swerve.getOdometry().getPoseMeters().getTranslation(),
        swerve.getOdometry().getPoseMeters().getRotation()),
      new PathPoint(targetPose.getTranslation(), targetPose.getRotation()));
    
    SwerveTrajectory.PathPlannerRunner(tagPos, swerve, swerve.getOdometry(), swerve.getPose().getRotation());
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

  private Pose3d getTagPos(int tagID) {
    double tagX = 0.0;
    double tagY = 0.0;
    double tagZ = 0.0;
    Rotation3d rotation = new Rotation3d(0,0,0);

    switch (tagID) {
      case 1:
        tagX = Constants.AlignmentConstants.TAG_1_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_1_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_1_POSE.getZ();
        rotation = Constants.AlignmentConstants.TAG_1_POSE.getRotation();
        break;

      case 2:
        tagX = Constants.AlignmentConstants.TAG_2_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_2_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_2_POSE.getZ();
        rotation = Constants.AlignmentConstants.TAG_2_POSE.getRotation();
        break;

      case 3:
        tagX = Constants.AlignmentConstants.TAG_3_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_3_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_3_POSE.getZ();
        rotation = Constants.AlignmentConstants.TAG_3_POSE.getRotation();
        break;

      case 6:
        tagX = Constants.AlignmentConstants.TAG_7_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_7_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_7_POSE.getZ();
        rotation = Constants.AlignmentConstants.TAG_7_POSE.getRotation();
        break;

      case 7:
        tagX = Constants.AlignmentConstants.TAG_8_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_8_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_8_POSE.getZ();
        rotation = Constants.AlignmentConstants.TAG_8_POSE.getRotation();
        break;

      case 8: 
        tagX = Constants.AlignmentConstants.TAG_8_POSE.getX();
        tagY = Constants.AlignmentConstants.TAG_8_POSE.getY();
        tagZ = Constants.AlignmentConstants.TAG_8_POSE.getZ();
        rotation = Constants.AlignmentConstants.TAG_8_POSE.getRotation();
        break;
    }
    return new Pose3d(tagX, tagY, tagZ, rotation);
  }

  public void setTagID(int tagID) {
    this.tagID = tagID;
  }

  public int getTagID() {
    return tagID;
  }

}