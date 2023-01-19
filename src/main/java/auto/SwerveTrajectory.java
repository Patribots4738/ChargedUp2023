package auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import hardware.Swerve;
import io.github.oblarg.oblog.Loggable;
import math.Constants;

public class SwerveTrajectory implements Loggable {

  // Create config for trajectory
  private static double timetrajectoryStarted;
  private static String trajectoryStatus="";
  private static SwerveTrajectory SINGLE_INSTANCE = new SwerveTrajectory();    

  public static double elapsedTime;

  public static SwerveTrajectory getInstance(){
      return SINGLE_INSTANCE;
  }


  /**
   * Returns a new HolonomicDriveController with constant PID gains.
   * 
   * The 2 PID controllers are controllers that should correct 
   * for error in the field-relative x and y directions respectively.
   * i.e. kXCorrectionP,I,D is PID for X correction
   *  and kYCorrectionP,I,D is PID for Y correction
   * 
   * The ProfiledPIDController for the rotation of the robot, 
   * utilizing a Trapezoidprofile for smooth locomotion in terms of max velocity and acceleration
   * 
   * @see kXCorrectionP The x-axis' P gain for the PID controller
   * @see kXCorrectionI The x-axis' I gain for the PID controller
   * @see kXCorrectionD The x-axis' D gain for the PID controller
   * @see kYCorrectionP The y-axis' P gain for the PID controller
   * @see kYCorrectionI The y-axis' I gain for the PID controller
   * @see kYCorrectionD The y-axis' D gain for the PID controller
   * @see kRotationCorrectionP The rotational-axis' P gain for the PID controller
   * @see kRotationCorrectionI The rotational-axis' I gain for the PID controller
   * @see kRotationCorrectionD The rotational-axis' D gain for the PID controller
   * @see kMaxAngularSpeedRadiansPerSecond The maximum velocity that the robot can TURN in the trapezoidal profile
   * @see kMaxAngularSpeedRadiansPerSecondSquared The maximum acceleration that the robot can TURN in the trapezoidal profile
   * @return A new HolonomicDriveController with the given PID gains (xP, xI, xD, yP, yI, yD, rotP, rotI, rotD) and constraints (maxVel, maxAccel)
   */
  public static HolonomicDriveController HDC = new HolonomicDriveController(
    new PIDController(Constants.AutoConstants.kXCorrectionP, Constants.AutoConstants.kXCorrectionI, Constants.AutoConstants.kXCorrectionD),
    new PIDController(Constants.AutoConstants.kYCorrectionP, Constants.AutoConstants.kYCorrectionI, Constants.AutoConstants.kYCorrectionD),
    new ProfiledPIDController(Constants.AutoConstants.kRotationCorrectionP, Constants.AutoConstants.kRotationCorrectionI, Constants.AutoConstants.kRotationCorrectionD,
      new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)));
  

  /**This is WPILIBs Trajectory Runner (docs.wpilib.org), it pretends that your robot is NOT a swerve drive.  This will work, but there are better options for 2022
   * @param _trajectory Pass in a trajectory that's stored in TrajectoryContainer
   * @param _odometry Pass in the robots odometry from SwerveDrive.java
   * @param _rotation2d Pass in the current angle of the robot
   */

  //Overload this method to accomdate different starting points, this can be useful when playing with multiple paths
  /**
   * This is PathPlanner.  It's awesome :) open up pathplanner.exe on the driverstation laptop.  Point the application to the locaiton of your coding project (must contain build.gradle).  Draw the path.  It will autosave. If everything is characterized correctly and your odometry reflects reality, ie. when the robot goes 1 meter it says it goes one meter--it will work like a charm.
   * @param _pathTraj run Pathplanner.loadpath("name of file without extension") pass it here
   * @param _odometry SwerveDrive.java's odometry
   * @param _rotation2d Pass in the current angle of the robot
   */
  public static void PathPlannerRunner(PathPlannerTrajectory _pathTraj, Swerve swerve, SwerveDriveOdometry _odometry, Rotation2d _rotation2d){
      elapsedTime = Timer.getFPGATimestamp()-timetrajectoryStarted;
      switch (trajectoryStatus) {
          case "setup":
              //swerve.resetOdometry(((PathPlannerState) _pathTraj.getInitialState()).poseMeters, ((PathPlannerState) _pathTraj.getInitialState()).poseMeters.getRotation()); 
              timetrajectoryStarted = Timer.getFPGATimestamp();
              trajectoryStatus = "execute";
              break;
          case "execute":
              
              if (elapsedTime <  ((PathPlannerState) _pathTraj.getEndState()).timeSeconds){
                  ChassisSpeeds _speeds = HDC.calculate(
                      _odometry.getPoseMeters(), 
                      ((PathPlannerState) _pathTraj.sample(elapsedTime)),((PathPlannerState) _pathTraj.sample(elapsedTime)).holonomicRotation);
                  swerve.drive(_speeds.vxMetersPerSecond,
                  _speeds.vyMetersPerSecond, 
                  _speeds.omegaRadiansPerSecond,false);
                  
              } else {
                  swerve.drive(0,0,0,false);
                  // swerve.setHoldRobotAngleSetpoint(((PathPlannerState) _pathTraj.getEndState()).holonomicRotation.getRadians());
                  trajectoryStatus = "done";

              }
              break;
          default:
              swerve.drive(0,0,0,false);
              break;
      }
  }

  public static void resetTrajectoryStatus(){
      trajectoryStatus = "setup";
  }
    
}