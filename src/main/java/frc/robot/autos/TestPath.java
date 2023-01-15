package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import hardware.Swerve;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class TestPath {

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public TestPath(Swerve swerve) {
        PathPlannerTrajectory testPath = PathPlanner.loadPath("New Path", 6, 3);
        PathPlannerState initialState = testPath.getInitialState();

        swerve.zeroHeading();
        swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
            initialState.holonomicRotation));
    }
}