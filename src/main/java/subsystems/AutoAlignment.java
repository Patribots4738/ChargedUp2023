package subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import org.photonvision.targeting.TargetCorner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import auto.AutoWaypoints;
import auto.SwerveTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import hardware.Swerve;
import math.Constants;


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
   * @param visionPose the position of the aprilTag relative to the bot
   * @param aprilTagID the ID of the tag being watched
   */
  public void calibrateOdometry(Pose2d visionPose, int aprilTagID) {
    
    double x = visionPose.getX();

    // flip the sign of y to correlate with pathplanner
    double y = -visionPose.getY();

    if (0 < tagID && tagID < 4) {
      x *= -1;
    }

    Pose2d targetPosition = getTagPos(aprilTagID);

    Pose2d generatedPose = new Pose2d(
        targetPosition.getX() + x, 
        targetPosition.getY() + y, 
        visionPose.getRotation().plus(targetPosition.getRotation())
    );
    
    swerve.resetOdometry(generatedPose);
    SwerveTrajectory.resetTrajectoryStatus();

  }

  public void moveToTag(int tagID, HolonomicDriveController HDC, AutoWaypoints autoWaypoints) {

    autoWaypoints.autoPeriodic();

    Pose2d targetPose = getTagPos(tagID);

    
    // if (0 < tagID && tagID < 4) {
      //   targetPose = targetPose.plus(new Transform2d(new Translation2d(-Units.inchesToMeters(15), 0), new Rotation2d(0)));
      // } else {
        //   targetPose = targetPose.plus(new Transform2d(new Translation2d(Units.inchesToMeters(15), 0), new Rotation2d(0)));
        // }
        
    int direction = (0 < tagID && tagID < 5) ? -1 : 1;

    targetPose = new Pose2d(
      new Translation2d(
        targetPose.getX() + ((Units.inchesToMeters(15) + (Constants.AlignmentConstants.kCameraPosition)) * direction), 
        targetPose.getY()
      ),
      targetPose.getRotation()
    );

    PathPlannerTrajectory tagPos = PathPlanner.generatePath(
      new PathConstraints(0.1, 0.1),
      new PathPoint(swerve.getOdometry().getPoseMeters().getTranslation(),
        swerve.getOdometry().getPoseMeters().getRotation()),
      new PathPoint(targetPose.getTranslation(), targetPose.getRotation()));
    
    SwerveTrajectory.PathPlannerRunner(tagPos, swerve, swerve.getOdometry(), swerve.getPose().getRotation());
    
    System.out.println("Direction: " + direction);
    System.out.println("Modified Target Pose: " + targetPose);
    System.out.println("Current Pose: " + swerve.getOdometry().getPoseMeters() + "\n\n");
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
    Rotation2d rotation = new Rotation2d(0);

    switch (tagID) {
      case 1:
        tagX = Constants.AlignmentConstants.kTag_1_pos.getX();
        tagY = Constants.AlignmentConstants.kTag_1_pos.getY();
        rotation = Constants.AlignmentConstants.kTag_1_pos.getRotation();
        break;

      case 2:
        tagX = Constants.AlignmentConstants.kTag_2_pos.getX();
        tagY = Constants.AlignmentConstants.kTag_2_pos.getY();
        rotation = Constants.AlignmentConstants.kTag_2_pos.getRotation();
        break;

      case 3:
        tagX = Constants.AlignmentConstants.kTag_3_pos.getX();
        tagY = Constants.AlignmentConstants.kTag_3_pos.getY();
        rotation = Constants.AlignmentConstants.kTag_3_pos.getRotation();
        break;

      case 6:
        tagX = Constants.AlignmentConstants.kTag_6_pos.getX();
        tagY = Constants.AlignmentConstants.kTag_6_pos.getY();
        rotation = Constants.AlignmentConstants.kTag_6_pos.getRotation();
        break;

      case 7:
        tagX = Constants.AlignmentConstants.kTag_7_pos.getX();
        tagY = Constants.AlignmentConstants.kTag_7_pos.getY();
        rotation = Constants.AlignmentConstants.kTag_7_pos.getRotation();
        break;

      case 8: 
        tagX = Constants.AlignmentConstants.kTag_8_pos.getX();
        tagY = Constants.AlignmentConstants.kTag_8_pos.getY();
        rotation = Constants.AlignmentConstants.kTag_8_pos.getRotation();
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