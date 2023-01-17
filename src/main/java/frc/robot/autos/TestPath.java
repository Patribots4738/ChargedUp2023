package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import hardware.Swerve;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class TestPath {

  PathPlannerTrajectory testPath;

  /**
   * Autonomous that aligns limelight then executes a trajectory.
   *
   * @param swerve swerve subsystem
   */
  public TestPath(Swerve swerve) {

    testPath = PathPlanner.loadPath("Square", 3, 1);
    // ben = PathPlanner.loadPath("Ben", 3, 1);

    PathPlannerState initialState = testPath.getInitialState();

    swerve.zeroHeading();
    swerve.resetOdometry(
      new Pose2d(initialState.poseMeters.getTranslation(),
      initialState.holonomicRotation));
  
  }

  public Trajectory getPath() {

    return this.testPath;

  }
}