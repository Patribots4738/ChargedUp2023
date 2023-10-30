// Kudos to https://youtu.be/IKOGwoJ2HLk for the theory!
package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.util.Constants.ArmConstants;
import frc.robot.util.Constants.PlacementConstants;

public class ArmCalculations {

    /**
     * Get the offset of the second arm relative to arm 1
     *
     * @param x the absolute x position from the first pivot point to position the
     *          claw
     * @param y the absolute y position from the first pivot point to position the
     *          claw
     *          Please keep in mind the x and y value must be under
     *          Constants.kMaxReachX,Y respectivly
     *          Due to an axis controlling the range, they will not go over
     * @return the angle to set the motor to, in radians
     */
    public double getUpperAngle(double x, double y) {
        double upperAngle = (Math.acos(
                (((Math.pow(x, 2)) + (Math.pow(y, 2))) -
                        ((Math.pow(ArmConstants.LOWER_ARM_LENGTH, 2)) +
                                (Math.pow(ArmConstants.UPPER_ARM_LENGTH, 2))))
                        /
                        (2 * (ArmConstants.LOWER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH))));

        return (upperAngle * -1);
    }

    /**
     * Get the offset of the first arm relative to the robot
     *
     * @param x the absolute x position from the first pivot point to position the
     *          claw
     * @param y the absolute y position from the first pivot point to position the
     *          claw
     *          Please keep in mind the x and y value must be under
     *          Constants.kMaxReachX,Y respectivly
     *          Due to an axis controlling the range, they will not go over
     */
    public double getLowerAngle(double x, double y, double q2) {
        double alpha = Math.atan2(
                (ArmConstants.UPPER_ARM_LENGTH * Math.sin(q2)),
                (ArmConstants.LOWER_ARM_LENGTH + (ArmConstants.UPPER_ARM_LENGTH * Math.cos(q2))));

        double q1 = ((Math.atan2(y, x) - alpha));

        return q1;
    }

    /**
     * A util function to linearly interpolate between two values
     * 
     * @param a the minimum value to interpolate between
     * @param b the maximum value to interpolate between
     * @param f the number to interpolate between the two values
     * @return the interpolated value
     */
    public double lerp(double a, double b, double f) {
        return (a + f * (b - a));
    }

    public static Trajectory generateHighConeTrajectory() {

        var startPos = new Pose2d(PlacementConstants.STOWED_POSITION, Rotation2d.fromDegrees(45));
        var endPos = new Pose2d(PlacementConstants.HIGH_CONE_PREP, Rotation2d.fromDegrees(180));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(PlacementConstants.MID_CONE_PREP);

        TrajectoryConfig config = new TrajectoryConfig(800, 800);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                startPos,
                interiorWaypoints,
                endPos,
                config);

        return trajectory;
    }

    public static Trajectory generateHighCubeTrajectory() {

        var startPos = new Pose2d(PlacementConstants.STOWED_POSITION, Rotation2d.fromDegrees(45));
        var endPos = new Pose2d(PlacementConstants.CUBE_HIGH, Rotation2d.fromDegrees(180));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(PlacementConstants.MID_CONE_PREP);

        TrajectoryConfig config = new TrajectoryConfig(800, 800);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                startPos,
                interiorWaypoints,
                endPos,
                config);

        return trajectory;
    }

    public static Trajectory generateHighPlacementTrajectory() {

        var startPos = new Pose2d(PlacementConstants.STOWED_POSITION, Rotation2d.fromDegrees(45));
        var endPos = new Pose2d(PlacementConstants.HIGH_CONE_POSITION_2, Rotation2d.fromDegrees(180));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(PlacementConstants.MID_CONE_PREP);
        interiorWaypoints.add(PlacementConstants.HIGH_CONE_PREP);
        interiorWaypoints.add(PlacementConstants.HIGH_CONE_POSITION_0);

        TrajectoryConfig config = new TrajectoryConfig(800, 800);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                startPos,
                interiorWaypoints,
                endPos,
                config);

        return trajectory;
    }

    public static Trajectory generateHighCubeToStowTrajectory() {

        var startPos = new Pose2d(PlacementConstants.CUBE_HIGH, Rotation2d.fromDegrees(0));
        var endPos = new Pose2d(PlacementConstants.STOWED_POSITION, Rotation2d.fromDegrees(50));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(PlacementConstants.MID_CONE_PREP);

        TrajectoryConfig config = new TrajectoryConfig(400, 400);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                startPos,
                interiorWaypoints,
                endPos,
                config);

        return trajectory;
    }

    public static Trajectory generateHighToStowTrajectory() {

        var startPos = new Pose2d(PlacementConstants.HIGH_RETRACT, Rotation2d.fromDegrees(0));
        var endPos = new Pose2d(PlacementConstants.STOWED_POSITION, Rotation2d.fromDegrees(50));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(PlacementConstants.MID_CONE_PREP);

        TrajectoryConfig config = new TrajectoryConfig(200, 200);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                startPos,
                interiorWaypoints,
                endPos,
                config);

        return trajectory;
    }

    public static Trajectory generateMidConeTrajectory() {

        var startPos = new Pose2d(PlacementConstants.STOWED_POSITION, Rotation2d.fromDegrees(45));
        var endPos = new Pose2d(PlacementConstants.MID_CONE_PREP, Rotation2d.fromDegrees(135));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(PlacementConstants.TRANSITION_POSITION);

        TrajectoryConfig config = new TrajectoryConfig(800, 800);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                startPos,
                interiorWaypoints,
                endPos,
                config);

        return trajectory;
    }
}