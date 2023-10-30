// Referenced from https://github.com/Stampede3630/2022-Code/blob/MK3Practice/src/main/java/frc/robot/SwerveTrajectory.java
package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.FieldConstants;
import edu.wpi.first.wpilibj.Timer;

public class SwerveTrajectory extends CommandBase {

    private final PathPlannerTrajectory trajectory;
    private final Swerve swerve;
    private final boolean longWait;
    private final Timer elapsedTime;

    public SwerveTrajectory(PathPlannerTrajectory pathTraj, Swerve swerve, boolean longWait) {
        trajectory = pathTraj;
        this.swerve = swerve;
        this.longWait = longWait;
        elapsedTime = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        elapsedTime.restart();
    }

    @Override
    public void execute() {
        // If the path has not completed time wise
        boolean shouldMirror = (FieldConstants.ALLIANCE == Alliance.Red) && (FieldConstants.GAME_MODE == FieldConstants.GameMode.AUTONOMOUS);
        // System.out.printf("Elapsed Time %.3f\n", elapsedTime - pathTraj.getTotalTimeSeconds());

        PathPlannerState state = (PathPlannerState) trajectory.sample(elapsedTime.get());
        PathPlannerState mirroredState = new PathPlannerState();

        // Create a new pathplannerstate based on the mirrored state's position
        // and taking the mirrored state's rotation and adding 180 degrees
        if (shouldMirror) {

        mirroredState.timeSeconds                       = state.timeSeconds;
        mirroredState.velocityMetersPerSecond           = state.velocityMetersPerSecond;
        mirroredState.accelerationMetersPerSecondSq     = state.accelerationMetersPerSecondSq;
        mirroredState.poseMeters                        = new Pose2d(
            (FieldConstants.FIELD_WIDTH_METERS - state.poseMeters.getTranslation().getX()),
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
        ChassisSpeeds speeds = AutoConstants.HDC.calculate(
            swerve.getPose(),
            // Pass in the alliance to flip on the Y if on red alliance
            (shouldMirror) 
                ? mirroredState
                : state,
            (shouldMirror) 
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
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.get() < (trajectory.getTotalTimeSeconds() + (FieldConstants.ALLIANCE == Alliance.Red || longWait ? 1 : 0.6));
    }
    
}
