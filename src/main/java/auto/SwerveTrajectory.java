// Refrenced from https://github.com/Stampede3630/2022-Code/blob/MK3Practice/src/main/java/frc/robot/SwerveTrajectory.java
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
import debug.*;

public class SwerveTrajectory implements Loggable {

    // Create config for trajectory
    public static double timetrajectoryStarted;
    public static String trajectoryStatus = "";
    public static SwerveTrajectory SINGLE_INSTANCE = new SwerveTrajectory();

    public static double elapsedTime;

    public static SwerveTrajectory getInstance() {
        return SINGLE_INSTANCE;
    }

    /**
     * Returns a new HolonomicDriveController with constant PID gains.
     * <p>
     * The 2 PID controllers are controllers that should correct
     * for error in the field-relative x and y directions respectively.
     * i.e. kXCorrectionP,I,D is PID for X correction
     * and kYCorrectionP,I,D is PID for Y correction
     * <p>
     * The ProfiledPIDController for the rotation of the robot,
     * utilizing a Trapezoidprofile for smooth locomotion in terms of max velocity and acceleration
     *
     * @return A new HolonomicDriveController with the given PID gains (xP, xI, xD, yP, yI, yD, rotP, rotI, rotD) and constraints (maxVel, maxAccel)
     */
    public static HolonomicDriveController HDC = new HolonomicDriveController(
            new PIDController(Constants.AutoConstants.kXCorrectionP, Constants.AutoConstants.kXCorrectionI, Constants.AutoConstants.kXCorrectionD),
            new PIDController(Constants.AutoConstants.kYCorrectionP, Constants.AutoConstants.kYCorrectionI, Constants.AutoConstants.kYCorrectionD),
            new ProfiledPIDController(Constants.AutoConstants.kRotationCorrectionP, Constants.AutoConstants.kRotationCorrectionI, Constants.AutoConstants.kRotationCorrectionD,
                    new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)));

    /**
     * This is PathPlanner.
     * It's awesome :)
     * Open up pathplanner.exe on the driverstation laptop.
     * Point the application to the location of your coding project (must contain build.gradle).
     * Draw the path.
     * It will autosave.
     * If everything is characterized correctly and your odometry reflects reality,
     * i.e. when the robot goes 1 meter, it says it went one meter--
     * it will work like a charm.
     *
     * @param _pathTraj   run Pathplanner.loadpath("name of file without an extension") pass it here
     * @param _odometry   SwerveDrive.java's odometry
     * @param _rotation2d Pass in the current angle of the robot
     */
    public static void PathPlannerRunner(PathPlannerTrajectory _pathTraj, Swerve swerve, SwerveDriveOdometry _odometry, Rotation2d _rotation2d) {

        elapsedTime = Timer.getFPGATimestamp() - timetrajectoryStarted;

        switch (trajectoryStatus) {

            case "setup":
                timetrajectoryStarted = Timer.getFPGATimestamp();
                trajectoryStatus = "execute";
                break;

            case "execute":

                Debug.debugPeriodic(
                        _pathTraj.sample(elapsedTime).poseMeters.getX() - _odometry.getPoseMeters().getX(),
                        _pathTraj.sample(elapsedTime).poseMeters.getY() - _odometry.getPoseMeters().getY(),
                        _pathTraj.sample(elapsedTime).poseMeters.getRotation().getDegrees() - _odometry.getPoseMeters().getRotation().getDegrees());

                // If the path has not completed time wise
                if (elapsedTime < _pathTraj.getEndState().timeSeconds + 5) {

                    // Use elapsedTime as a refrence for where we NEED to be
                    // Then, sample the position and rotation for that time,
                    // And calculate the ChassisSpeeds required to get there
                    ChassisSpeeds _speeds = HDC.calculate(
                            _odometry.getPoseMeters(),
                            _pathTraj.sample(elapsedTime),
                            ((PathPlannerState) _pathTraj.sample(elapsedTime)).holonomicRotation);

                    // Set the states for the motor using calculated values above
                    // It is important to note that fieldRelative is false,
                    // but calculations make it so it is true i.e. rotation is independant
                    // (This is seen 6-5 lines above)
                    swerve.drive(
                            _speeds.vxMetersPerSecond,
                            _speeds.vyMetersPerSecond,
                            _speeds.omegaRadiansPerSecond, false);

                } else {

                    swerve.drive(0, 0, 0, false);
                    trajectoryStatus = "done";

                }

                break;

            default:

                swerve.drive(0, 0, 0, false);
                break;

        }
    }

    public static void resetTrajectoryStatus() {

        trajectoryStatus = "setup";

    }

    public static HolonomicDriveController getHDC() {

        return HDC;

    }

}