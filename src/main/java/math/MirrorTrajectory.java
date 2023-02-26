package math;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class MirrorTrajectory extends PathPlannerTrajectory {

  public MirrorTrajectory() {
    super();
  }

  @Override
  public static PathPlannerState transformStateForAlliance(
      PathPlannerState state, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      // Create a new state so that we don't overwrite the original
      PathPlannerState transformedState = new PathPlannerState();

      Translation2d transformedTranslation =
          new Translation2d(state.poseMeters.getX(), FIELD_WIDTH_METERS - state.poseMeters.getY());
      Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
      Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1);

      transformedState.timeSeconds = state.timeSeconds;
      transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
      transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
      transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
      transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
      transformedState.holonomicRotation = transformedHolonomicRotation;
      transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
      transformedState.curveRadius = -state.curveRadius;
      transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
      transformedState.deltaPos = state.deltaPos;

      return transformedState;
    } else {
      return state;
    }
  }
}
