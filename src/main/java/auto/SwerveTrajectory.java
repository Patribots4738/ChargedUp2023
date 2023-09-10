// Referenced from https://github.com/Stampede3630/2022-Code/blob/MK3Practice/src/main/java/frc/robot/SwerveTrajectory.java
package auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import hardware.Swerve;
import io.github.oblarg.oblog.Loggable;
import calc.Constants;
import calc.Constants.AlignmentConstants;

public class SwerveTrajectory implements Loggable {

  // Create config for trajectory
  public static double timeTrajectoryStarted;
  // The trajectoryStatus represents what stage of auto we are on
  // i.e. setting up ("setup"), executing ("execute"), or finished ("done")
  public static String trajectoryStatus = "";

  // The amount of time that has elapsed in the path...
  // This variable is used like a timer slider so we progress with the path
  // at the same time as the path...
  // A rough example of this is seen in the pathplanner's "play" functionality,
  // where the robot does a little movement
  // we try to replicate that using this and Trajectory.sample(elapsedTime);
  public static double elapsedTime;

  /**
   * Returns a new HolonomicDriveController with constant PID gains.
   * <p>
   * The 2 PID controllers are controllers that should correct
   * for error in the field-relative x and y directions respectively.
   * i.e. X_CORRECTION_P,I,D is PID for X correction
   * and Y_CORRECTION_P,I,D is PID for Y correction
   * <p>
   * The ProfiledPIDController for the rotation of the robot,
   * utilizing a Trapezoidprofile for smooth locomotion in terms of max velocity and acceleration
   * To tune: <p>
   * If you overshoot on the X, add less P for X. If you undershoot, add more P for X.
   * As of 2/27/23 I have found little use for D.
   * If you rotate too much, then reduce the P for theta.
   * D can be seen as beneficial here as the rotation is allowed more speed than position.
   */

  // IF YOU EVER WANT TO TUNE THIS, HERE IS HOW:
  // look at the commented code below teehee
  public static HolonomicDriveController HDC = new HolonomicDriveController(
      new PIDController(
        Constants.AutoConstants.X_CORRECTION_P,
        Constants.AutoConstants.X_CORRECTION_I,
        Constants.AutoConstants.X_CORRECTION_D),
      new PIDController(
        Constants.AutoConstants.Y_CORRECTION_P,
        Constants.AutoConstants.Y_CORRECTION_I,
        Constants.AutoConstants.Y_CORRECTION_D),
      new ProfiledPIDController(
        Constants.AutoConstants.ROTATION_CORRECTION_P,
        Constants.AutoConstants.ROTATION_CORRECTION_I,
        Constants.AutoConstants.ROTATION_CORRECTION_D,
          new TrapezoidProfile.Constraints(
            Constants.AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            Constants.AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)));

  // IF YOU EVER WANT TO TUNE THIS, READ BELOW:
  // IF YOU EVER WANT TO TUNE THIS, READ BELOW:
  // IF YOU EVER WANT TO TUNE THIS, READ BELOW:
  // the @Config annotation here will save you many many redeploys.
  // It allows you to change the values of the constants in the shuffleboard
  // IN SHUFFLEBOARD, IN REAL TIME, REAL
  // no more redeploys, no more redeploys, no more redeploys
  // Have... fun? :) - Alexander Hamilton, 2023

  // @Config
  // public static double xP = Constants.AutoConstants.X_CORRECTION_P;
  // @Config
  // public static double xI = Constants.AutoConstants.X_CORRECTION_I;
  // @Config
  // public static double xD = Constants.AutoConstants.X_CORRECTION_D;
  // @Config
  // public static double yP = Constants.AutoConstants.Y_CORRECTION_P;
  // @Config
  // public static double yI = Constants.AutoConstants.Y_CORRECTION_I;
  // @Config
  // public static double yD = Constants.AutoConstants.Y_CORRECTION_D;
  // @Config
  // public static double rotP = Constants.AutoConstants.ROTATION_CORRECTION_P;
  // @Config
  // public static double rotI = Constants.AutoConstants.ROTATION_CORRECTION_I;
  // @Config
  // public static double rotD = Constants.AutoConstants.ROTATION_CORRECTION_D;
  // @Config
  // public static double maxAngularSpeed = Constants.AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  // @Config
  // public static double maxAngularAccel = Constants.AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED;
  
  // public static HolonomicDriveController HDC = new HolonomicDriveController(
  //   new PIDController(xP,
  //                     xI,
  //                     xD),
  //   new PIDController(yP,
  //                     yI,
  //                     yD),
  //   new ProfiledPIDController(rotP,
  //                             rotI,
  //                             rotD,
  //                             new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAccel)));

  /**
   * Pathplanner time!
   * Open up pathplanner.exe on the driverstation laptop.
   * Point the application to the location of your coding project (must contain build.gradle).
   * Draw the path.
   * It will autosave.
   * If everything is characterized correctly and your odometry reflects reality,
   * i.e. when the robot goes 1 meter, it says it went one meter--
   * You only need to tune the Holonomic Drive Controller, which is explained above...
   *
   * @param pathTraj run Pathplanner.loadpath("name of file without an extension") pass it here
   * @param swerve the current instance of swerve
   */
  public static void PathPlannerRunner(PathPlannerTrajectory pathTraj, Swerve swerve, boolean longWait) {

    elapsedTime = Timer.getFPGATimestamp() - timeTrajectoryStarted;

    switch (trajectoryStatus) {

      case "setup":
        timeTrajectoryStarted = Timer.getFPGATimestamp();
        trajectoryStatus = "execute";
        break;

      case "execute":

        // If the path has not completed time wise
        if (elapsedTime < (pathTraj.getTotalTimeSeconds() + (DriverStation.getAlliance() == DriverStation.Alliance.Red || longWait ? 1 : 0.6)))
        {
          // System.out.printf("Elapsed Time %.3f\n", elapsedTime - pathTraj.getTotalTimeSeconds());

          PathPlannerState state = (PathPlannerState) pathTraj.sample(elapsedTime);
          PathPlannerState mirroredState = new PathPlannerState();

          // Create a new pathplannerstate based on the mirrored state's position
          // and taking the mirrored state's rotation and adding 180 degrees
          if ((DriverStation.getAlliance() == DriverStation.Alliance.Red) && DriverStation.isAutonomous()) {

            mirroredState.timeSeconds                       = state.timeSeconds;
            mirroredState.velocityMetersPerSecond           = state.velocityMetersPerSecond;
            mirroredState.accelerationMetersPerSecondSq     = state.accelerationMetersPerSecondSq;
            mirroredState.poseMeters                        = new Pose2d(
              (AlignmentConstants.FIELD_WIDTH_METERS - state.poseMeters.getTranslation().getX()),
                state.poseMeters.getTranslation().getY(),
                state.poseMeters.getRotation().unaryMinus().plus(Rotation2d.fromDegrees(Math.PI))
              );
            mirroredState.curvatureRadPerMeter              = state.curvatureRadPerMeter;
            mirroredState.holonomicRotation                 = state.holonomicRotation.plus(Rotation2d.fromRadians(Math.PI)).unaryMinus();
            mirroredState.angularVelocityRadPerSec          = state.angularVelocityRadPerSec;
            mirroredState.holonomicAngularVelocityRadPerSec = state.holonomicAngularVelocityRadPerSec;

          }
          
          // Use elapsedTime as a refrence for where we NEED to be
          // Then, sample the position and rotation for that time,
          // And calculate the ChassisSpeeds required to get there
          ChassisSpeeds speeds = HDC.calculate(
              swerve.getPose(),
              // Pass in the alliance to flip on the Y if on red alliance
              ((DriverStation.getAlliance() == DriverStation.Alliance.Red) && DriverStation.isAutonomous()) 
                  ? mirroredState
                  : state,
              ((DriverStation.getAlliance() == DriverStation.Alliance.Red) && DriverStation.isAutonomous()) 
                  ? mirroredState.holonomicRotation 
                  : state.holonomicRotation);

          // Set the states for the motor using calculated values above
          // It is important to note that fieldRelative is false,
          // but calculations make it, so it is true i.e. rotation is independent
          // (This is seen 6-5 lines above)
          swerve.drive(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond, false, false);

        } else {

          swerve.drive(0, 0, 0, false, true);
          trajectoryStatus = "done";

        }

        break;

      default:

        swerve.drive(0, 0, 0, false, true);
        break;

    }
  }

  public static void resetTrajectoryStatus() {

    trajectoryStatus = "setup";

  }
}
