package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DriverUI;
import frc.robot.subsystems.PhotonCameraUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.ClawConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.PlacementConstants;
import frc.robot.util.Constants.VisionConstants;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAlignment extends CommandBase {

    /**
     * A visual representation of the apriltag positions
     * / --------------------------------------------- \
     * 5                      |                        4
     * |                      |                        |
     * |                      |                        |
     * 6                      |                        3
     * |                      |                        |
     * 7                      |                        2
     * |                      |                        |
     * 8                      |                        1
     * \ --------------------------------------------- /
     */

    Swerve swerve;
    PhotonCameraUtil photonVision;

    private int tagID;
    private int coneOffset = 0;
    private int substationOffset = -1;
    private double startedChargePad = -1;
    private Timer alignmentTimer = new Timer();

    // This variable is used to tell us how far away we currently are from an april
    // tag
    private double currentNorm = -1;

    public AutoAlignment(Swerve swerve, PhotonCameraUtil photonVision) {
        this.swerve = swerve;
        this.photonVision = photonVision;

        addRequirements(swerve, photonVision);
    }

    /**
     * Calibrate the odometry for the swerve
     */
    public void calibrateOdometry() {

        // Create an "Optional" object that contains the estimated pose of the robot
        // This can be present (see's tag) or not present (does not see tag)
        Optional<EstimatedRobotPose> result = photonVision.getEstimatedRobotPose();

        // If the result of the estimatedRobotPose exists, and the skew of the tag is
        // less than 3 degrees (to prevent false results)
        if (result.isPresent()) {
            EstimatedRobotPose camEstimatedPose = result.get();
            // Add the vision measurement to the pose estimator to update the odometry
            swerve.getPoseEstimator().addVisionMeasurement(
                    camEstimatedPose.estimatedPose.toPose2d(),
                    Timer.getFPGATimestamp() - VisionConstants.LATENCY);
        }

        if (photonVision.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
            // Get the target pose (the pose of the tag we want to go to)
            Pose2d targetPose = photonVision.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
            targetPose = getModifiedTargetPose(targetPose);
            currentNorm = swerve.getPose().minus(targetPose).getTranslation().getNorm();
        }
    }

    public void setNearestAlignmentOffset() {

        if (!PlacementConstants.CONE_MODE) {
            this.coneOffset = 0;
            return;
        }

        if (photonVision.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {

            Pose2d aprilTagPose2d = photonVision.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
            double tagXOffset = getTagXOffset();
            double tagYOffset = getTagYOffset();

            double negativeOffsetNorm = swerve.getPose().minus(aprilTagPose2d.plus(new Transform2d(
                    new Translation2d(
                            tagXOffset,
                            -tagYOffset),
                    new Rotation2d()))).getTranslation().getNorm();

            double positiveOffsetNorm = swerve.getPose().minus(aprilTagPose2d.plus(new Transform2d(
                    new Translation2d(
                            tagXOffset,
                            tagYOffset),
                    new Rotation2d()))).getTranslation().getNorm();

            /*
             * If we are on red alliance, left of the tag is going to be the negative on the
             * Y axis
             * Due to this.moveToTag() checking if we are on the blue alliance and flipping
             * the sign,
             * we need to flip the sign here
             */
            if (negativeOffsetNorm < positiveOffsetNorm) {

                // If we are looking at a human player tag,
                // Then we have been evaluating for the substationOffset
                if (tagID == 4 || tagID == 5) {
                    this.substationOffset = -1;
                } else {
                    this.coneOffset = -1 * ((FieldConstants.ALLIANCE == Alliance.Blue) ? 1 : -1);
                }
            } else /* (positiveOffsetNorm < negativeOffsetNorm) */ {

                // If we are looking at a human player tag,
                // Then we have been evaluating for the substationOffset
                if (tagID == 4 || tagID == 5) {
                    this.substationOffset = 1;
                } else {
                    this.coneOffset = ((FieldConstants.ALLIANCE == Alliance.Blue) ? 1 : -1);
                }
            }
        }
    }

    /**
     * Align to the tag we are closest to
     * Only do the Y and Theta axis,
     * as we will manually move the robot to the X axis
     */
    public ChassisSpeeds getAutoAlignChassisSpeeds() {

        // If we cannot see a tag
        if (tagID == 0) {
            return new ChassisSpeeds(0, 0, 0);
        }
        Pose2d targetPose = swerve.getPose();

        // Check if our tagID is valid... (Assume it is for logic purposes)
        if (photonVision.aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
            // Get the target pose (the pose of the tag we want to go to)
            targetPose = photonVision.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
        }

        // If we are on the left side of the field: we need to add the grid offset +
        // cone/substation offset
        // If we are on the right side of the field: we need to subtract the grid offset
        // + cone/substation offset
        // If we are going to a substation: we need to add the substation offset instead
        // of the cone offset
        // We add the grid length to both because we still want to be a small bit away
        // from the tag
        // There is a bit of a logic issue that the else statement "should" be
        // subtracting, but it doesn't work when you do that...
        // oh well.
        targetPose = getModifiedTargetPose(targetPose);

        double adjustedY = targetPose.getY() - swerve.getPose().getY();

        MathUtil.applyDeadband(adjustedY, (PlacementConstants.CONE_BASE_RADIUS));

        adjustedY += swerve.getPose().getY();

        return AutoConstants.HDC.calculate(
                swerve.getPose(),
                new Pose2d(
                        swerve.getPose().getX(),
                        adjustedY,
                        targetPose.getRotation().plus(Rotation2d.fromDegrees(180))),
                // Notice the 0 m/s here
                // This is because we want the robot to end at a stop,
                // This might be funky when we manually drive it on the X axis
                0,
                targetPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public void setNearestValues() {
        this.setTagID(getNearestTag());
        this.setNearestAlignmentOffset();
    }

    /**
     * Get the tag nearest to the robot using its position
     * while using the alliance color to factor out tags
     * 
     * @return the nearest tag to the bot that is for the same alliance
     */
    public int getNearestTag() {

        // A reminder that tag 0 sets this.moveToTag() to return;
        int nearestTag = 0;
        double nearestDistance = 1000;
        double currentDistance = 1000;

        Translation2d currentPosition = swerve.getPose().getTranslation();

        // If we are on the blue alliance, only look at positions of tags 8,7,6 and 4
        if (FieldConstants.ALLIANCE == Alliance.Blue) {
            for (int i = 8; i > 4; i--) {
                // Tag 5 is for the red alliance's substation, not ours
                // Skip over to our substation
                if (i == 5) {
                    i = 4;
                }

                // This if a statement prevents the robot from crashing if we input an absurd
                // tag ID,
                // but it should be assumed that the tag location is present.
                if (photonVision.aprilTagFieldLayout.getTagPose(i).isPresent()) {
                    currentDistance = currentPosition.getDistance(
                            photonVision.aprilTagFieldLayout.getTagPose(i).get().toPose2d().getTranslation());
                }
                if (currentDistance < nearestDistance) {
                    nearestDistance = currentDistance;
                    nearestTag = i;
                }
            }
        }
        // We are on the red alliance, only look at positions of tags 1,2,3 and 5
        else {
            for (int i = 1; i < 5; i++) {
                // Tag 4 is for the blue alliance's substation, not ours
                // Skip over to our substation
                if (i == 4) {
                    i = 5;
                }

                // This if a statement prevents the robot from crashing if we input an absurd
                // tag ID,
                // but it should be assumed that the tag location is present.
                if (photonVision.aprilTagFieldLayout.getTagPose(i).isPresent()) {
                    currentDistance = currentPosition.getDistance(
                            photonVision.aprilTagFieldLayout.getTagPose(i).get().toPose2d().getTranslation());
                }
                if (currentDistance < nearestDistance) {
                    nearestDistance = currentDistance;
                    nearestTag = i;
                }
            }
        }

        return nearestTag;
    }

    private double getTagXOffset() {
        return (tagID == 4 || (tagID == 5)) ?
        // Tag 4/5 means we need to subtract the grid barrier
        // since it is on the right side of the field
                (Units.inchesToMeters(PlacementConstants.HUMAN_TAG_PICKUP.getX() - ClawConstants.CLAW_LENGTH_INCHES)) :
                // Otherwise, we need to add the grid barrier
                (FieldConstants.GRID_BARRIER_METERS + (PlacementConstants.ROBOT_LENGTH_METERS / 2)
                        + PlacementConstants.BUMPER_LENGTH_METERS);
    }

    private double getTagYOffset() {
        // Check if we are applying a substation offset or a cone offset
        return (tagID == 4 || tagID == 5) ? FieldConstants.SUBSTATION_OFFSET_METERS : FieldConstants.CONE_OFFSET_METERS;
    }

    /**
     * Get the modified target pose based on the alliance color
     * 
     * @param targetPose the target pose of the tag we want to go to
     * @return the modified target pose using constants for grid/substation
     */
    public Pose2d getModifiedTargetPose(Pose2d targetPose) {
        targetPose = targetPose.plus(new Transform2d(
                new Translation2d(
                        getTagXOffset(),
                        (tagID == 4 || tagID == 5) ? (FieldConstants.SUBSTATION_OFFSET_METERS * this.substationOffset)
                                : (FieldConstants.CONE_OFFSET_METERS * this.coneOffset)
                                        * ((FieldConstants.ALLIANCE == Alliance.Blue) ? 1 : -1)),
                Rotation2d.fromDegrees(180)));
        return targetPose;
    }

    public int getTagID() {
        return tagID;
    }

    public void setTagID(int tagID) {
        this.tagID = tagID;
    }

    public int getConeOffset() {
        return this.coneOffset;
    }

    public void setConeOffset(int coneOffset) {

        int previousConeOffset = this.coneOffset;

        // If we are on cone mode, skip cone offsets of 0
        // also, skip straight to the next cone if we are on the same cone
        // in other words,
        // if we are on cone mode, at cone index 1, and we want to go to cone index 2,
        // skip cone index 0
        // or,
        // if we are *not* on cone mode, (at cone index 0) and we want to go to cone
        // index 1, skip cone index 1 and go to 2.
        // This is because the call from robot.java will run as
        // (setConeOffset(getConeOffset +/- 1))
        if (PlacementConstants.CONE_MODE) {
            if (previousConeOffset == -1) {
                if (coneOffset == 0) {
                    coneOffset = 1;
                }
            } else if (previousConeOffset == 1) {
                if (coneOffset == 0) {
                    coneOffset = -1;
                }
            }
        } else {
            this.coneOffset = 0;
            if (coneOffset == -1) {
                coneOffset = -2;
            } else if (coneOffset == 1) {
                coneOffset = 2;
            }
        }

        // Pan the coneOffset to the next tag if it is able to do so
        // It cannot do so if there is no grid in the desired direction
        if (coneOffset < -1) {
            if (tagID == 2 || tagID == 3) {
                this.tagID--;
                coneOffset = (PlacementConstants.CONE_MODE) ? 1 : 0;
                // System.out.println("Case 1: Tag = " + tagID + ", coneOffset = " +
                // coneOffset);
            } else if (tagID == 6 || tagID == 7) {
                this.tagID++;
                coneOffset = (PlacementConstants.CONE_MODE) ? 1 : 0;
                // System.out.println("Case 2: Tag = " + tagID + ", coneOffset = " +
                // coneOffset);
            }
        } else if (coneOffset > 1) {
            if (tagID == 1 || tagID == 2) {
                this.tagID++;
                coneOffset = (PlacementConstants.CONE_MODE) ? -1 : 0;
                // System.out.println("Case 3: Tag = " + tagID + ", coneOffset = " +
                // coneOffset);
            } else if (tagID == 7 || tagID == 8) {
                this.tagID--;
                coneOffset = (PlacementConstants.CONE_MODE) ? -1 : 0;
                // System.out.println("Case 4: Tag = " + tagID + ", coneOffset = " +
                // coneOffset);
            }
        }

        // Clamp the cone offset to -1, 0, or 1
        this.coneOffset = MathUtil.clamp(coneOffset, -1, 1);

        // If we are not on cone mode, ensure the cone offset is zero
        if (!PlacementConstants.CONE_MODE) {
            this.coneOffset = 0;
        }

        // If we actually changed indexes, reset the auto alignment status,
        // so we can re-align to the new index
        if (previousConeOffset != this.coneOffset) {
            // SwerveTrajectory.resetTrajectoryStatus();
        }
    }

    public Command coneOffsetLeft() {
        return Commands.runOnce(() -> {
                if (getTagID() == 4 || getTagID() == 5) {
                    setSubstationOffset((FieldConstants.ALLIANCE == Alliance.Blue) ? 1 : -1);
                }
                else {
                    setConeOffset(getConeOffset() + ((FieldConstants.ALLIANCE == Alliance.Blue) ? 1 : -1));
                }
            }
        );
    }

    public Command coneOffsetRight() {
        return Commands.runOnce(() -> {
                if (getTagID() == 4 || getTagID() == 5) {
                    setSubstationOffset((FieldConstants.ALLIANCE == Alliance.Blue) ? -1 : 1);
                }
                else {
                    setConeOffset(getConeOffset() + ((FieldConstants.ALLIANCE == Alliance.Blue) ? -1 : 1));
                }
            }
        );
    }

    public void resetTimer() {
        alignmentTimer.restart();
    }

    public double getAlignmentTimer() {
        return alignmentTimer.get();
    }

    public int getSubstationOffset() {
        return this.substationOffset;
    }

    public void setSubstationOffset(int substationOffset) {
        this.substationOffset = MathUtil.clamp(substationOffset, -1, 1);
    }

    public void setConeMode(boolean coneMode) {
        PlacementConstants.CONE_MODE = coneMode;
        DriverUI.coneMode = PlacementConstants.CONE_MODE;
        // If the cone offset is 0, and we are switching to cone mode,
        // set the cone offset to 1 (closest to human tag)
        if (!PlacementConstants.CONE_MODE) {
            this.coneOffset = 0;
        }
    }

    public Command setConeModeTrue() {
        return Commands.runOnce(() -> setConeMode(true));
    }

    public Command setConeModeFalse() {
        return Commands.runOnce(() -> setConeMode(false));
    }

    public double getCurrentNorm() {
        return this.currentNorm;
    }

    public Trigger getAlignedTrigger(DoubleSupplier threshold) {
        return new Trigger(() -> 
            getCurrentNorm() < (threshold.getAsDouble()) && (getCurrentNorm() != -1)
        );
    }

    public double getAngleSnapThetaSpeed(Rotation2d desiredAngle) {

        // Use a Holonomic Drive Controller to calculate the speeds for the robot
        return AutoConstants.HDC.getThetaController().calculate(
            swerve.getYaw().getRadians(),
            desiredAngle.getRadians());

    }
}