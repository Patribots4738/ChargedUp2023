package subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Logger;
import hardware.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import auto.SwerveTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import hardware.Swerve;
import io.github.oblarg.oblog.Loggable;
import math.Constants;
import debug.*;

public class AutoAllignment {
  
  Swerve swerve;

  SwerveDriveOdometry odometry;

  Pose2d targetPose;

  public AutoAllignment(Swerve swerve){
    this.swerve = swerve;
  }

  public void callibrateOdometry(Pose2d aprilPose, int aprilTagID){
    double x = aprilPose.getX();
    double y = aprilPose.getY();
    int tagID = aprilTagID;

    Pose2d targetPosition = getTagPos(tagID);

    Pose2d generatedPos = new Pose2d(targetPosition.getX() + x, targetPosition.getY() + y, new Rotation2d(targetPosition.getRotation().getDegrees()));

    swerve.resetOdometry(generatedPos);

  }

  public void moveToTag(int tagID, HolonomicDriveController HDC, double time){
    
    // PathPlannerTrajectory tagPos = PathPlanner.generatePath(
    //   new PathConstraints(0.5, 0.5),
    //   new PathPoint(swerve.getOdometry().getPoseMeters().getTranslation(), swerve.getOdometry().getPoseMeters().getRotation(), swerve.getOdometry().getPoseMeters().getRotation()),
    //   new PathPoint()

    // );

    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
      new PathConstraints(4, 3),
      new PathPoint(swerve.getOdometry().getPoseMeters().getTranslation(), swerve.getOdometry().getPoseMeters().getRotation()),
      new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45))
    );

    SwerveTrajectory.PathPlannerRunner(traj1, swerve, swerve.getOdometry(), swerve.getPose().getRotation());
        
    // targetPose = getTagPos(tagID);
    // State targetState = new State(1, 0, 0, targetPose, 0);
    // ChassisSpeeds speeds = HDC.calculate(
    //           swerve.getOdometry().getPoseMeters(), 
    //           (targetState),
    //           (targetPose.getRotation()));
            
            
    //         swerve.drive(speeds.vxMetersPerSecond*0.25,
    //         speeds.vyMetersPerSecond*0.25, 
    //         speeds.omegaRadiansPerSecond,false);
    // return targetState;
  }

  public void moveRelative(double x, double y, double rotation, HolonomicDriveController HDC) {
    Pose2d currentPose = swerve.getOdometry().getPoseMeters();
    
    Pose2d targetPose = new Pose2d(
                                    currentPose.getX() + x,
                                    currentPose.getY() + y,
                                    new Rotation2d(currentPose.getY() + rotation)
                                  );

    State targetState = new State(0, 0, 0, targetPose, 0);

    ChassisSpeeds speeds = HDC.calculate(
                                          currentPose,
                                          targetState,
                                          targetPose.getRotation()
                                        );

    swerve.drive(speeds.vxMetersPerSecond*0.25,
    speeds.vyMetersPerSecond*0.25, 
    speeds.omegaRadiansPerSecond,false);
  }

  public boolean isAlligned(){
    if (swerve.getOdometry().getPoseMeters() == targetPose){
      return true;
    }
    return false;
  }

  
  private Pose2d getTagPos(int tagID){
    double tagX = 0.0;
    double tagY = 0.0;
    Rotation2d rotation = new Rotation2d(0);

    switch (tagID){
      case 1:
        tagX = Constants.AllignmentConstants.kTag_1_pos.getX();
        tagY = Constants.AllignmentConstants.kTag_1_pos.getY();
        rotation = Constants.AllignmentConstants.kTag_1_pos.getRotation();
        break;

      case 2:
        tagX = Constants.AllignmentConstants.kTag_2_pos.getX();
        tagY = Constants.AllignmentConstants.kTag_2_pos.getY();
        rotation = Constants.AllignmentConstants.kTag_2_pos.getRotation();
        break;

      case 3:
        tagX = Constants.AllignmentConstants.kTag_3_pos.getX();
        tagY = Constants.AllignmentConstants.kTag_3_pos.getY();
        rotation = Constants.AllignmentConstants.kTag_3_pos.getRotation();
        break;

      case 6:
        tagX = Constants.AllignmentConstants.kTag_6_pos.getX();
        tagY = Constants.AllignmentConstants.kTag_6_pos.getY();
        rotation = Constants.AllignmentConstants.kTag_6_pos.getRotation();
        break;

      case 7:
        tagX = Constants.AllignmentConstants.kTag_7_pos.getX();
        tagY = Constants.AllignmentConstants.kTag_7_pos.getY();
        rotation = Constants.AllignmentConstants.kTag_7_pos.getRotation();
        break;

      case 8: 
        tagX = Constants.AllignmentConstants.kTag_8_pos.getX();
        tagY = Constants.AllignmentConstants.kTag_8_pos.getY();
        rotation = Constants.AllignmentConstants.kTag_8_pos.getRotation();
        break;
    }
    return new Pose2d(1000.0, tagY, rotation);
  }
}