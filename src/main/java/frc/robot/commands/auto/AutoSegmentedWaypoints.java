// Referenced from https://github.com/Stampede3630/2022-Code/blob/0ad2aa434f50d8f5dc93e965809255f697dadffe/src/main/java/frc/robot/AutoSegmentedWaypoints.java#L81
package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriverUI;
import frc.robot.commands.AutoAlignment;
import frc.robot.commands.SwerveTrajectory;
import frc.robot.commands.auto.AutoPathStorage.AutoPose;
import frc.robot.commands.auto.AutoPathStorage.Waypoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.PlacementConstants;

public class AutoSegmentedWaypoints extends CommandBase {

    Swerve swerve;
    Arm arm;
    Claw claw;
    SwerveTrajectory swerveTrajectoryCommand;
    AutoAlignment autoAlignmentCommand;

    public Waypoint[] chosenWaypoints;
    public AutoPose chosenAutoPath;

    public int currentWaypointNumber = 0;

    public double autoDelay;

    public boolean stateHasInitialized = false;
    public boolean clawHasStarted = false;
    public boolean hasMovedArm = false;
    public boolean startedChargePad = false;
    public boolean armInit = false;
    public boolean halfway = false;

    public AutoSegmentedWaypoints(Swerve swerve, Arm arm, Claw claw, AutoAlignment autoAlignmentCommand,
            AutoPose chosenAutoPath) {
        // Good ol' references to subsystems :>
        this.swerve = swerve;
        this.arm = arm;
        this.claw = claw;
        this.chosenAutoPath = chosenAutoPath;
        chosenWaypoints = this.chosenAutoPath.getWaypointSet();
        this.autoAlignmentCommand = autoAlignmentCommand;

        addRequirements(swerve, arm, claw);
    }

    @Override
    public void initialize() {

        boolean mirrored = false;

        currentWaypointNumber = 0;

        PathPlannerState initialPathPose = chosenWaypoints[0].getPathPlannerSegment().getInitialState();
        // Make a new initial pose, and set it to a different reference value
        PathPlannerState mirroredInitialPathPose = new PathPlannerState();

        // Create a new pathplannerstate based on the mirrored state's position
        // and taking the mirrored state's rotation and adding 180 degrees
        if ((FieldConstants.ALLIANCE == Alliance.Red)
                && (FieldConstants.GAME_MODE == FieldConstants.GameMode.AUTONOMOUS)) {

            mirrored = true;

            mirroredInitialPathPose.timeSeconds = initialPathPose.timeSeconds;
            mirroredInitialPathPose.velocityMetersPerSecond = initialPathPose.velocityMetersPerSecond;
            mirroredInitialPathPose.accelerationMetersPerSecondSq = initialPathPose.accelerationMetersPerSecondSq;
            mirroredInitialPathPose.poseMeters = new Pose2d(
                    (FieldConstants.FIELD_WIDTH_METERS - initialPathPose.poseMeters.getTranslation().getX()),
                    initialPathPose.poseMeters.getTranslation().getY(),
                    initialPathPose.poseMeters.getRotation().unaryMinus().plus(Rotation2d.fromDegrees(Math.PI)));
            mirroredInitialPathPose.curvatureRadPerMeter = initialPathPose.curvatureRadPerMeter;
            mirroredInitialPathPose.holonomicRotation = initialPathPose.holonomicRotation
                    .plus(Rotation2d.fromRadians(Math.PI)).unaryMinus();
            mirroredInitialPathPose.angularVelocityRadPerSec = initialPathPose.angularVelocityRadPerSec;
            mirroredInitialPathPose.holonomicAngularVelocityRadPerSec = initialPathPose.holonomicAngularVelocityRadPerSec;

        }

        // Scrumptious boolean resets
        swerveTrajectoryCommand = new SwerveTrajectory(chosenWaypoints[0].getPathPlannerSegment(), swerve, false);
        swerveTrajectoryCommand.schedule();
        stateHasInitialized = false;
        clawHasStarted = false;
        hasMovedArm = false;
        startedChargePad = false;
        armInit = false;
        halfway = false;

        swerve.resetOdometry(new Pose2d(
                (mirrored) ? mirroredInitialPathPose.poseMeters.getTranslation()
                        : initialPathPose.poseMeters.getTranslation(),
                (mirrored) ? mirroredInitialPathPose.holonomicRotation : initialPathPose.holonomicRotation));

    }

    @Override
    public void execute() {
        waypointRunner(chosenWaypoints);
    }

    /**
     * This method moves subsystems around during auto
     * <p>
     * If both arms are in the set position, then
     * stop the auto path and move on to the next waypoint
     */
    private void setArmIndex(int armIndex, double clawSpeed) {

        // Limit the claw speed based on what object we plan on placing
        // This will reduce the possibility of popping a cube, for example.
        if (clawSpeed == PlacementConstants.CLAW_OUTTAKE_SPEED_CONE && !clawHasStarted) {
            switch (armIndex) {
                case PlacementConstants.CONE_HIGH_PLACEMENT_INDEX:
                case PlacementConstants.CONE_MID_PLACEMENT_INDEX:
                    claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CONE);
                    break;
                default:
                    // If our auto path contains "pickup and place a cube",
                    // Then set the claw speed to intake cubes
                    claw.setDesiredSpeed(
                            ((chosenAutoPath.getName().contains("A_2") || chosenAutoPath.getName().contains("D_8"))
                                    ? PlacementConstants.CLAW_INTAKE_SPEED_CUBE
                                    : PlacementConstants.CLAW_INTAKE_SPEED_CONE));
                    break;
            }
        }

        // Check if the arm is ready to move to the next waypoint mid-path
        if (!swerveTrajectoryCommand.isFinished() && currentWaypointNumber != 0 && arm.getAtDesiredPositions()) {

            // This is where the arm will "prep" to go to placement locations
            // Very useful for saving time
            if ((((FieldConstants.ALLIANCE == Alliance.Blue && swerve.getPose().getX() < 4) ||
                    (FieldConstants.ALLIANCE == Alliance.Red
                            && swerve.getPose().getX() > (FieldConstants.FIELD_WIDTH_METERS - 3))))
                    &&
                    halfway &&
                    /*
                     * Get if we are on a pickup -> charge path,
                     * and if we are on the last waypoint,
                     * don't move the arm
                     * 
                     * In other words, break out of this if statement
                     */
                    !((chosenAutoPath.getName().contains("A_CH") ||
                            chosenAutoPath.getName().contains("B_CH") ||
                            chosenAutoPath.getName().contains("C_CH") ||
                            chosenAutoPath.getName().contains("D_CH")) &&
                            currentWaypointNumber == chosenWaypoints.length - 1)) {
                // Prepare the arm for the next waypoint before the path is done
                switch (armIndex) {
                    // If the next waypoint is a high cone placement, prepare the arm for a high
                    // cone placement
                    case PlacementConstants.CONE_HIGH_PLACEMENT_INDEX:
                        arm.setArmIndex(PlacementConstants.CONE_HIGH_PREP_INDEX);
                        arm.startTrajectory(PlacementConstants.HIGH_CONE_TRAJECTORY);
                        break;
                    // Tuen the high cube index into a traj
                    case PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX:
                        arm.setArmIndex(armIndex);
                        arm.startTrajectory(PlacementConstants.HIGH_CUBE_TRAJECTORY);
                        break;
                    // If the next waypoint is a mid cone placement, prepare the arm for a mid cone
                    // placement
                    case PlacementConstants.CONE_MID_PLACEMENT_INDEX:
                        arm.setArmIndex(PlacementConstants.CONE_MID_PREP_INDEX);
                        break;
                    // If the next waypoint is nothing that requires a prep, just go to the position
                    default:
                        arm.setArmIndex(armIndex);
                        break;
                }
            }
            // This is where the arm will "prep" to go to pickup locations
            // Very useful for saving time
            else if (((FieldConstants.ALLIANCE == Alliance.Blue && swerve.getPose().getX() > 5) ||
                    (FieldConstants.ALLIANCE == Alliance.Red
                            && swerve.getPose().getX() < (FieldConstants.FIELD_WIDTH_METERS - 5)))) {
                PlacementConstants.CONE_MODE = false;
                halfway = true;

                // Only set the claw to the intake position if it is not already there
                // Small redundancy prevention to make sure we don't overload our runtimes
                /// me when i one time boolean but not other time boolean
                if (arm.getArmIndex() != PlacementConstants.CUBE_INTAKE_INDEX &&
                        arm.getArmIndex() != PlacementConstants.AUTO_CUBE_INTAKE_INDEX) {
                    arm.setArmIndex(PlacementConstants.CUBE_INTAKE_INDEX);
                }
                claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CUBE);
            }
        }

        // Once the robot is in position...
        else if (swerveTrajectoryCommand.isFinished()) {
            if ((chosenAutoPath.getName().contains("A_2") || chosenAutoPath.getName().contains("D_8"))
                    && currentWaypointNumber == 1) {
                hasMovedArm = true;
            }
            // If the arm is not in the desired position, move it
            if (!hasMovedArm) {
                // If the arm is at a prep index, change the desired index to be the other half
                // of that prep index...
                // Notice that the floor intake does not have a second half,
                // this is because the first transition point is the prep index's end point
                if (arm.getAtPrepIndex()) {
                    arm.finishPlacement();
                }
                // The arm is not at a prep index...
                else {
                    arm.setArmIndex(armIndex);
                    if (armIndex == PlacementConstants.CONE_HIGH_PLACEMENT_INDEX && currentWaypointNumber == 0) {
                        arm.setArmIndex(PlacementConstants.CONE_HIGH_PREP_INDEX);
                        arm.startTrajectory(PlacementConstants.HIGH_PLACE_TRAJECTORY);
                    }
                }
                // Prevent the arm from setting the index again
                hasMovedArm = true;
            }
        }

        // Once the arm and robot are in position...
        if ((swerveTrajectoryCommand.isFinished() && arm.getAtDesiredPositions())) {

            // Move the claw to the desired speed
            if (!clawHasStarted && (clawSpeed != PlacementConstants.CLAW_STOPPED_SPEED)) {
                // Set autoDelay to the current time,
                // This is to allow the claw to move for a bit before moving on.
                autoDelay = Timer.getFPGATimestamp();
                claw.setDesiredSpeed(clawSpeed);
                // Prevent the autoDelay from being reset
                clawHasStarted = true;
            }

            // 0.2 seconds since the claw has moved (and if there are more waypoints)
            if ((Timer.getFPGATimestamp() - autoDelay > 0.15 || currentWaypointNumber == 0)
                    || (clawSpeed == PlacementConstants.CLAW_STOPPED_SPEED)) {
                swerveTrajectoryCommand.end(true);
            }
        }
    }

    public void waypointRunner(Waypoint[] thisWaypointSet) {
        // If we made one round with the state, we have successfully initialized
        if (!stateHasInitialized) {
            if (currentWaypointNumber == 0) {
                swerveTrajectoryCommand.end(true);
            } else {
                // Only run the path if the robot isn't trying to align
                // This will give priority to the alignment
                if (!startedChargePad) {
                    swerveTrajectoryCommand = new SwerveTrajectory(
                            thisWaypointSet[currentWaypointNumber].getPathPlannerSegment(), swerve,
                            chosenAutoPath.getName().contains("1H_A_2H"));
                    swerveTrajectoryCommand.schedule();
                }
            }
            stateHasInitialized = true;
        }

        // During the path, loop through our periodic loop
        // notice the "this."
        // it makes sure we don't call Arm.java's "setArmIndex()"
        if (!swerveTrajectoryCommand.isFinished()) {
            this.setArmIndex(thisWaypointSet[currentWaypointNumber].getArmPosIndex(),
                    thisWaypointSet[currentWaypointNumber].getClawDirection());
        }

        // If there are not more waypoints, tell the robot to level itself
        // We can do this if we want to go on charged or not,
        // because if we did not want, to then the robot will already be level.
        if ((currentWaypointNumber == chosenWaypoints.length - 1 &&
                (swerveTrajectoryCommand.isFinished()))) {
            if (!startedChargePad) {
                autoAlignmentCommand.schedule();
                // Prevent startChargePad from being called again
                startedChargePad = true;
            }
            // The autoalignment command is being scheduled... yey.
            // Run the charge pad leveling PID loop
            arm.setArmIndex(PlacementConstants.STOWED_INDEX);
        }
        // We are not done with all of our auto segments,
        // but we are done with one...
        else if (swerveTrajectoryCommand.isFinished()) {

            clawHasStarted = false;
            hasMovedArm = false;

            // If our arm was at a high-placement index,
            // send it to stow while staying away from the grid
            // using the high-to-stow index
            if (arm.getArmIndex() == PlacementConstants.CONE_HIGH_PLACEMENT_INDEX ||
                    arm.getArmIndex() == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX ||
                    arm.getArmIndex() == PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX) {
                arm.setArmIndex(PlacementConstants.HIGH_TO_STOWED_INDEX);
                arm.startTrajectory(
                        (arm.getArmIndex() == PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX)
                                ? PlacementConstants.HIGH_CUBE_TO_STOWED_TRAJECTORY
                                : PlacementConstants.HIGH_CONE_TO_STOWED_TRAJECTORY);
            }
            // If our path placed a cube, just stow if we are not placing at
            // high_cube_launch
            // The transition point in stow is enough to stay away from the arena
            // Keep the arm out if we are at the cube intake index
            // (only will trigger on legacy auto paths)
            else if (arm.getArmIndex() != PlacementConstants.CUBE_INTAKE_INDEX) {
                arm.setArmIndex(PlacementConstants.STOWED_INDEX);
            }

            // If we are not done with all of our waypoints,
            // move on to the next one
            if (currentWaypointNumber < chosenWaypoints.length - 1) {
                currentWaypointNumber++;

                // If we are not on the first waypoint,
                // and we are doing a pickup-to-charge path,
                // Keep the claw intaking while we go to the pad
                // This is because under the hood in some paths,
                // The robot will say "A_CHARGE",
                // But really go from A to MOBILITY to CHARGE
                // And for those paths we want to keep intaking
                if (currentWaypointNumber > 1) {
                    if (chosenAutoPath.getName().contains("A_CH") ||
                            chosenAutoPath.getName().contains("B_CH") ||
                            chosenAutoPath.getName().contains("C_CH") ||
                            chosenAutoPath.getName().contains("D_CH")) {
                        // We can assume that we are holding a cube
                        // Since our claw is so strong
                        // that we can hold cones without moving the claw at all,
                        // and it wouldn't harm to use this speed for cones,
                        // but it would harm to use the cone speed for cubes.
                        claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CUBE);
                        clawHasStarted = true;
                    } else {
                        // Only set halfway to true if...
                        // we are doing legacy auto path,
                        // Else, reset it to continue the prep on the arm.
                        // We can check this by seeing if the path name has "_" on the end.
                        halfway = !(chosenAutoPath.getName().endsWith("_"));
                    }
                }
                stateHasInitialized = false;
            }

            // If we are on waypoint 0 or 1,
            // (since this will also run right after 0 is over),
            // we can assume we just placed a game piece...
            // just in case there are issues, this is a failsafe
            // in case the game piece is not placed
            if ((currentWaypointNumber != 1) && (currentWaypointNumber != 0)) {
                // Only move the claw before the arm
                // if it needs to hold a game piece
                if ((thisWaypointSet[currentWaypointNumber]
                        .getClawDirection() != PlacementConstants.CLAW_OUTTAKE_SPEED_CONE) &&
                /*
                 * Lastly, if we just picked up a game piece and are going to the charge pad,
                 * we want to keep the game piece in the claw
                 * so... do NOT stop the claw if going to charge from a pickup
                 * 
                 * Also, be sure to check if startedChargePad is false...
                 * since it triggers after the path is completed,
                 * And if we were to only check the waypoint number,
                 * it would be at the "final waypoint index"
                 * before the robot is actually on the charge pad.
                 * 
                 * This would cause the claw to stop before the robot
                 * is actually on the charge pad, and thus,
                 * the arm would be in the way of the robot charging.
                 * 
                 * 
                 * In other words, if we started the charge pad,
                 * and we are on an A/B/C/D_CHARGE path,
                 * then break out of this if statement
                 */
                        !(startedChargePad &&
                                (chosenAutoPath.getName().contains("A_CH") ||
                                        chosenAutoPath.getName().contains("B_CH") ||
                                        chosenAutoPath.getName().contains("C_CH") ||
                                        chosenAutoPath.getName().contains("D_CH")))) {
                    // Me when the biggest if statement,
                    // in all of code,
                    // has a singular commented out line
                    // claw.stopClaw();
                }
            }
        }
    }
}