package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmCalculations;
import frc.robot.util.Constants.ArmConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.PlacementConstants;
import frc.robot.util.Pose3dLogger;

// Uncomment the following lines to enable logging
// Mainly used for creating/editing arm position constants
// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;

// We use the black solution as seen in: https://www.desmos.com/calculator/fqyyldertp
public class Arm extends SubsystemBase {

    int armPosDimension1 = PlacementConstants.STOWED_INDEX;
    int armPosDimension2 = 1;

    private boolean startedTransition = false;

    private boolean operatorOverride = false;

    private double armXReference = 0;
    private double armYReference = 0;

    // The DESIRED rotation of the arms
    private double upperReferenceAngle = Units.degreesToRadians(200);
    private double lowerReferenceAngle = Units.degreesToRadians(122);

    private boolean armsAtDesiredPosition = false;

    // @Log
    private double armXPos = 0;
    // @Log
    private double armYPos = 0;

    private final CANSparkMax lowerArmRight;
    private final CANSparkMax lowerArmLeft;
    private final CANSparkMax upperArm;

    private final AbsoluteEncoder lowerArmEncoder;
    private final AbsoluteEncoder upperArmEncoder;
    private final SparkMaxPIDController lowerArmPIDController;
    private final SparkMaxPIDController upperArmPIDController;

    private boolean upperPIDSlot1;
    private boolean upperPIDSlot2;

    private final ArmCalculations armCalculations;
    private Trajectory currentTrajectory;
    private double[] trajectoryFinalAngles;
    private Timer trajectoryTimer;
    private boolean followingTrajectory;

    private double lowerArmEncoderSimPosition = 0;
    private double upperArmEncoderSimPosition = 0;

    private final Mechanism2d armDesiredMechanism = new Mechanism2d(
            Units.inchesToMeters(ArmConstants.MAX_REACH) * 2, Units.inchesToMeters(ArmConstants.MAX_REACH));
    private final MechanismRoot2d armRoot = armDesiredMechanism.getRoot(
            "Arm",
            Units.inchesToMeters(ArmConstants.MAX_REACH),
            Units.inchesToMeters(6));
    private final MechanismLigament2d lowerArmMechanism = armRoot.append(
            new MechanismLigament2d(
                    "lowerArm",
                    Units.inchesToMeters(ArmConstants.LOWER_ARM_LENGTH * 1.25),
                    lowerReferenceAngle));

    private final MechanismLigament2d upperArmMechanism = lowerArmMechanism.append(
            new MechanismLigament2d(
                    "upperArm",
                    Units.inchesToMeters(ArmConstants.UPPER_ARM_LENGTH * 1.25),
                    upperReferenceAngle));

    /**
     * Constructs a new Arm and configures the encoders and PID controllers.
     */
    public Arm() {

        lowerArmRight = new CANSparkMax(ArmConstants.LOWER_ARM_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        lowerArmLeft = new CANSparkMax(ArmConstants.LOWER_ARM_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        upperArm = new CANSparkMax(ArmConstants.UPPER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        lowerArmRight.setIdleMode(IdleMode.kBrake);
        lowerArmLeft.setIdleMode(IdleMode.kBrake);
        upperArm.setIdleMode(IdleMode.kCoast);

        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        lowerArmRight.restoreFactoryDefaults();
        lowerArmLeft.restoreFactoryDefaults();
        upperArm.restoreFactoryDefaults();

        lowerArmEncoder = lowerArmRight.getAbsoluteEncoder(Type.kDutyCycle);
        upperArmEncoder = upperArm.getAbsoluteEncoder(Type.kDutyCycle);

        lowerArmPIDController = lowerArmRight.getPIDController();
        upperArmPIDController = upperArm.getPIDController();

        lowerArmPIDController.setFeedbackDevice(lowerArmEncoder);
        upperArmPIDController.setFeedbackDevice(upperArmEncoder);

        lowerArmEncoder.setPositionConversionFactor(ArmConstants.LOWER_ENCODER_POSITION_FACTOR);
        lowerArmEncoder.setVelocityConversionFactor(ArmConstants.LOWER_ENCODER_VELOCITY_FACTOR);

        upperArmEncoder.setPositionConversionFactor(ArmConstants.UPPER_ENCODER_POSITION_FACTOR);
        upperArmEncoder.setVelocityConversionFactor(ArmConstants.UPPER_ENCODER_VELOCITY_FACTOR);

        lowerArmLeft.setSmartCurrentLimit(ArmConstants.LOWER_FREE_LIMIT);
        lowerArmRight.setSmartCurrentLimit(ArmConstants.LOWER_FREE_LIMIT);
        upperArm.setSmartCurrentLimit(ArmConstants.UPPER_FREE_LIMIT);

        // Set PID constants for the lower and upper SPARK MAX(s)
        lowerArmPIDController.setP(ArmConstants.LOWER_P);
        lowerArmPIDController.setI(ArmConstants.LOWER_I);
        lowerArmPIDController.setD(ArmConstants.LOWER_D);
        lowerArmPIDController.setFF(ArmConstants.LOWER_FF);
        lowerArmPIDController.setOutputRange(
                ArmConstants.LOWER_MIN_OUTPUT,
                ArmConstants.LOWER_MAX_OUTPUT);

        upperArmPIDController.setP(ArmConstants.UPPER_P, 0);
        upperArmPIDController.setI(ArmConstants.UPPER_I, 0);
        upperArmPIDController.setD(ArmConstants.UPPER_D, 0);
        upperArmPIDController.setFF(ArmConstants.UPPER_FF, 0);
        upperArmPIDController.setOutputRange(
                ArmConstants.UPPER_MIN_OUTPUT,
                ArmConstants.UPPER_MAX_OUTPUT, 0);

        // This is something that is not done with a lot of teams,
        // it is an additional PID gain set which is used in our
        // arm trajectory folowings, so that we tell the arm to go
        // super fast, but also be controlled when we want.
        upperArmPIDController.setP(ArmConstants.UPPER_P2, 1);
        upperArmPIDController.setI(ArmConstants.UPPER_I2, 1);
        upperArmPIDController.setD(ArmConstants.UPPER_D2, 1);
        upperArmPIDController.setFF(ArmConstants.UPPER_FF, 1);
        upperArmPIDController.setOutputRange(
                ArmConstants.UPPER_MIN_OUTPUT,
                ArmConstants.UPPER_MAX_OUTPUT, 1);

        // This is something that is not done with a lot of teams,
        // it is an additional PID gain set which is used in our
        // arm trajectory folowings, so that we tell the arm to go
        // super fast, but also be controlled when we want.
        upperArmPIDController.setP(ArmConstants.UPPER_P3, 2);
        upperArmPIDController.setI(ArmConstants.UPPER_I3, 2);
        upperArmPIDController.setD(ArmConstants.UPPER_D3, 2);
        upperArmPIDController.setFF(ArmConstants.UPPER_FF, 2);
        upperArmPIDController.setOutputRange(
                ArmConstants.UPPER_MIN_OUTPUT,
                ArmConstants.UPPER_MAX_OUTPUT, 2);

        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        lowerArmRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        lowerArmRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        upperArm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        upperArm.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);

        // Save the SPARK MAX configuration. If a SPARK MAX
        // browns out, it will retain the last configuration
        lowerArmLeft.follow(lowerArmRight, true);
        upperArmEncoder.setInverted(true);

        // zeroUpperArmEncoder();
        // zeroLowerArmEncoder();

        NeoMotorConstants.motors.add(lowerArmLeft);
        NeoMotorConstants.motors.add(lowerArmRight);
        NeoMotorConstants.motors.add(upperArm);

        armCalculations = new ArmCalculations();
        setBrakeMode();

        trajectoryTimer = new Timer();
        // This may not be necesary, but if we make our trajectory now it will be ready
        // for later
        // Trajectories take around 10ms to create, which may be
        currentTrajectory = PlacementConstants.HIGH_CONE_TRAJECTORY;

        logArmData();

    }

    public void periodic() {
        if (!operatorOverride && !followingTrajectory) {
            indexPeriodic();
        } else if (followingTrajectory) {
            trajectoryPeriodic();
        }

        if ((FieldConstants.IS_SIMULATION)) {
            if (Math.abs(lowerReferenceAngle - lowerArmEncoderSimPosition) > Units.degreesToRadians(3)) {
                lowerArmEncoderSimPosition += (lowerReferenceAngle - lowerArmEncoderSimPosition)/10;
            }
            if (Math.abs(upperReferenceAngle - upperArmEncoderSimPosition) > Units.degreesToRadians(3)) {
                upperArmEncoderSimPosition += (upperReferenceAngle - upperArmEncoderSimPosition)/10;
            }
        }

        // Check if we want to use secondary PID gains
        // Slot 1 is used to make the arm go super fast
        // Slot 2 is an in-between gain set which
        // slows the arm down just enough to not overshoot
        boolean upperPIDSlot1 = followingTrajectory && armPosDimension1 != PlacementConstants.CONE_MID_PREP_INDEX;
        boolean upperPIDSlot2 = (FieldConstants.GAME_MODE == FieldConstants.GameMode.TELEOP)
                && trajectoryTimer.hasElapsed(currentTrajectory.getTotalTimeSeconds() / 2);

        // On either PID slot boolean change
        // update both, then ask the SparkMAX to change its ff
        // We do this becuase we want to limit our communications with the spark
        // as much as possible, to reduce loop overruns.
        if (this.upperPIDSlot1 != upperPIDSlot1 || this.upperPIDSlot2 != upperPIDSlot2) {
            setUpperArmAngle(upperReferenceAngle);
        }

        // upperDiff = (Units.radiansToDegrees(upperReferenceAngle) -
        // Units.radiansToDegrees(getUpperArmAngle()));
        // lowerDiff = (Units.radiansToDegrees(lowerReferenceAngle) -
        // Units.radiansToDegrees(getLowerArmAngle()));
        // // Use forward kinematics to get the x and y position of the end effector
        armXPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.cos((getLowerArmAngle() - (Math.PI / 2))))
                + (ArmConstants.UPPER_ARM_LENGTH
                        * Math.cos((getUpperArmAngle() - Math.PI) + (getLowerArmAngle() - (Math.PI / 2)))));
        armYPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.sin((getLowerArmAngle() - (Math.PI / 2))))
                + (ArmConstants.UPPER_ARM_LENGTH
                        * Math.sin((getUpperArmAngle() - Math.PI) + (getLowerArmAngle() - (Math.PI / 2)))));

        logArmData();
    }

    public void trajectoryPeriodic() {

        boolean atDesiredFine = (Math
                .abs(trajectoryFinalAngles[0] - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_FINE &&
                Math.abs(trajectoryFinalAngles[1] - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_FINE);

        if (atDesiredFine) {
            startedTransition = true;
            armsAtDesiredPosition = true;
            followingTrajectory = false;
            trajectoryTimer.stop();
            // System.out.println(trajectoryTimer.get());
            return;
        }

        this.drive(currentTrajectory.sample(trajectoryTimer.get()).poseMeters.getTranslation());

    }

    public void startTrajectory(Trajectory trajectory) {

        this.currentTrajectory = trajectory;
        this.followingTrajectory = true;
        this.trajectoryTimer.restart();

        Translation2d finalPosition = currentTrajectory.sample(currentTrajectory.getTotalTimeSeconds()).poseMeters
                .getTranslation();

        double finalUpperAngle = armCalculations.getUpperAngle(finalPosition.getX(), finalPosition.getY());
        double finalLowerAngle = armCalculations.getLowerAngle(finalPosition.getX(), finalPosition.getY(),
                finalUpperAngle);

        // Add PI/2 to lowerArmAngle...
        // because the calculated angle is relative to the ground,
        // And the zero for the encoder is set to the direction of gravity
        // Add PI to upperArmAngle...
        // because armCalculations gives us the angle relative to the upper arm
        finalLowerAngle += (Math.PI / 2);
        finalUpperAngle += Math.PI;

        // Clamp the output angles as to not murder our precious hard stops
        finalUpperAngle = MathUtil.clamp(
                finalUpperAngle,
                ArmConstants.UPPER_ARM_LOWER_LIMIT,
                ArmConstants.UPPER_ARM_UPPER_LIMIT);

        // Clamp the output angles as to not murder our precious hard stops
        finalLowerAngle = MathUtil.clamp(
                finalLowerAngle,
                ArmConstants.LOWER_ARM_LOWER_LIMIT,
                ArmConstants.LOWER_ARM_UPPER_LIMIT);

        this.trajectoryFinalAngles = new double[] { finalLowerAngle, finalUpperAngle };

    }

    public void indexPeriodic() {

        boolean atDesiredCoarse = (Math
                .abs(lowerReferenceAngle - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_COARSE &&
                Math.abs(upperReferenceAngle - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_COARSE);

        boolean atDesiredFine = (Math
                .abs(lowerReferenceAngle - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_FINE &&
                Math.abs(upperReferenceAngle - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_FINE);

        boolean finalDeadband = (armPosDimension2 < PlacementConstants.ARM_POSITIONS[armPosDimension1].length - 1)
                ? atDesiredCoarse
                : atDesiredFine;

        // Syntax example:
        // PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]
        if (!startedTransition) {
            startedTransition = true;
            drive(PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]);
            return;
        }

        // Notice that this code is getting the difference in angle between the arms.
        // It might be better to instead use the difference in position, but I'm not
        // sure. - Hamilton
        if (finalDeadband && !armsAtDesiredPosition) {
            armPosDimension2++;

            // This line isn't strictly necessary, but could be included...
            // armPosDimension2 = MathUtil.clamp(armPosDimension2, 0,
            // PlacementConstants.ARM_POSITIONS[armPosDimension1].length);

            if (armPosDimension2 >= PlacementConstants.ARM_POSITIONS[armPosDimension1].length) {
                armsAtDesiredPosition = true;
                if (armPosDimension1 == PlacementConstants.ARM_FLIP_INDEX ||
                        armPosDimension1 == PlacementConstants.HIGH_TO_STOWED_INDEX ||
                        armPosDimension1 == PlacementConstants.MID_TO_STOWED_INDEX) {
                    armPosDimension1 = PlacementConstants.STOWED_INDEX;
                    armPosDimension2 = PlacementConstants.ARM_POSITIONS[PlacementConstants.STOWED_INDEX].length - 1;
                }
                return;
            }
            drive(PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]);
        }
    }

    /**
     * Sets arm index from the PlacementConstants.ARM_POSITIONS 2d array
     *
     * @param index the direction to change the arm index by
     */
    public void setArmIndex(int index) {

        this.followingTrajectory = false;

        index = MathUtil.clamp(index, 0, PlacementConstants.ARM_POSITIONS.length - 1);

        // Check if we are already at the desired index, and if we are not operator
        // overriding,
        // This is because, if we are operator overriding, we want to be able to go to
        // any index
        // If we are trying to go to a floor index, allow it to restart itself
        if ((index == armPosDimension1 ||
                (index == PlacementConstants.STOWED_INDEX &&
                        (armPosDimension1 == PlacementConstants.HIGH_TO_STOWED_INDEX ||
                                (armPosDimension1 == PlacementConstants.ARM_FLIP_INDEX ||
                                        armPosDimension1 == PlacementConstants.MID_TO_STOWED_INDEX
                                                && armsAtDesiredPosition))))
                &&
                index != PlacementConstants.CONE_FLIP_INDEX &&
                index != PlacementConstants.CONE_INTAKE_INDEX &&
                index != PlacementConstants.CUBE_INTAKE_INDEX &&
                !operatorOverride) {
            startedTransition = true;
            return;
        } else {
            startedTransition = false;
            armsAtDesiredPosition = false;
        }

        armPosDimension2 = 0;

        if (index == PlacementConstants.CONE_HIGH_PREP_INDEX &&
                armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX ||
                index == PlacementConstants.CONE_MID_PREP_INDEX &&
                        armPosDimension1 == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX
                ||
                (index == PlacementConstants.STOWED_INDEX &&
                        armPosDimension1 == PlacementConstants.CONE_FLIP_INDEX ||
                        armPosDimension1 == PlacementConstants.CONE_INTAKE_INDEX ||
                        armPosDimension1 == PlacementConstants.CUBE_INTAKE_INDEX)) {
            armPosDimension2 = PlacementConstants.ARM_POSITIONS[index].length - 1;
        }

        armPosDimension1 = index;
        // Turn off operator override to prevent arm.drive from setting values wrong
        this.operatorOverride = false;

    }

    public void setArmIndex(int index, boolean jumpToEnd) {

        setArmIndex(index);

        if (jumpToEnd) {
            armPosDimension1 = index;
            armPosDimension2 = PlacementConstants.ARM_POSITIONS[index].length - 1;
        }

    }

    public int getArmIndex() {
        return this.armPosDimension1;
    }

    public boolean halfwayFinishedWithConeFlip() {
        return this.armPosDimension1 == PlacementConstants.CONE_FLIP_INDEX &&
                this.armPosDimension2 >= PlacementConstants.ARM_POSITIONS[armPosDimension1].length - 1
                && !armsAtDesiredPosition;
    }

    /**
     * Calculate the position of the arm based on the joystick input
     * as an absolute position in inches, multiplied by
     * Constants.ArmConstants.kMaxReachX/Y respectively
     *
     * @param position either the joystick input or the desired absolute position
     *                 this case is handled under OperatorOverride
     */
    public void drive(Translation2d position) {

        // If operatorOverride is true, add the joystick input to the current position
        // recall that this value is in inches
        if (operatorOverride) {
            this.armXReference += (position.getX());
            this.armYReference += (position.getY());
        } else {
            this.armXReference = position.getX();
            this.armYReference = position.getY();
        }

        // Make sure armX and armY are within the range of 0 to infinity
        // Because we cannot reach below the ground.
        // Even though our arm starts 11 inches above the ground,
        // the claw is roughly 11 inches from the arm end
        armYReference = (armYReference < 0) ? 0 : armYReference;

        Translation2d armPosition = new Translation2d(armXReference, armYReference);

        // Proof: https://www.desmos.com/calculator/ppsa3db9fa
        // If the distance from zero is greater than the max reach, cap it at the max
        // reach
        if (armPosition.getNorm() > ArmConstants.MAX_REACH) {
            armPosition = armPosition.times((ArmConstants.MAX_REACH - 0.1) / armPosition.getNorm());
        }

        // If the distance from zero is less than the min reach, cap it at the min reach
        // This min reach is the lower arm length - the upper arm length
        else if (armPosition.getNorm() < ArmConstants.MIN_REACH) {
            armPosition = armPosition.times((ArmConstants.MIN_REACH + 0.1) / armPosition.getNorm());
        }

        // If the arm is trying to reach higher than 6'6", cap it at 6'6"
        // The field gives us this limit.
        if (armPosition.getY() > ArmConstants.MAX_REACH_Y) {
            armPosition = new Translation2d(armPosition.getX(), ArmConstants.MAX_REACH_Y);
        }
        // If the arm is trying to reach further than 48" out from the chassis, cap it
        // at 48"
        // The field gives us this limit.
        if (armPosition.getX() > ArmConstants.MAX_REACH_X) {
            armPosition = new Translation2d(ArmConstants.MAX_REACH_X, armPosition.getY());
        } else if (armPosition.getX() < -ArmConstants.MAX_REACH_X) {
            armPosition = new Translation2d(-ArmConstants.MAX_REACH_X, armPosition.getY());
        }

        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armPosition.getX(), armPosition.getY());
        double lowerArmAngle = armCalculations.getLowerAngle(armPosition.getX(), armPosition.getY(), upperArmAngle);

        // If upperArmAngle is NaN, then tell the arm not to change position
        // We only check upperArmAngle because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
            System.out.println("Upper angle NAN " + armPosition + " " + armPosition.getNorm());
            return;
        }

        upperArmMechanism.setAngle(Units.radiansToDegrees(upperArmAngle));
        lowerArmMechanism.setAngle(Units.radiansToDegrees(lowerArmAngle));

        // Add PI/2 to lowerArmAngle...
        // because the calculated angle is relative to the ground,
        // And the zero for the encoder is set to the direction of gravity
        // Add PI to upperArmAngle...
        // because armCalculations gives us the angle relative to the upper arm
        lowerArmAngle += (Math.PI / 2);
        upperArmAngle += Math.PI;

        // Clamp the output angles as to not murder our precious hard stops
        upperArmAngle = MathUtil.clamp(
                upperArmAngle,
                ArmConstants.UPPER_ARM_LOWER_LIMIT,
                ArmConstants.UPPER_ARM_UPPER_LIMIT);

        // Clamp the output angles as to not murder our precious hard stops
        lowerArmAngle = MathUtil.clamp(
                lowerArmAngle,
                ArmConstants.LOWER_ARM_LOWER_LIMIT,
                ArmConstants.LOWER_ARM_UPPER_LIMIT);

        // Set the reference values to the modified X and Y values
        // This is especially important for the operatorOverride going OOB
        this.armXReference = armPosition.getX();
        this.armYReference = armPosition.getY();

        // Finally, set the reference values for the lower and upper arm:
        setLowerArmReference(lowerArmAngle);
        setUpperArmReference(upperArmAngle);
    }

    public void setLowerArmReference(double reference) {
        this.lowerReferenceAngle = reference;
        setLowerArmAngle(lowerReferenceAngle);
    }

    public void setUpperArmReference(double reference) {
        this.upperReferenceAngle = reference;
        setUpperArmAngle(upperReferenceAngle);
    }

    /**
     * Set the position of an arm
     *
     * @param angle the position to set the upper arm to
     *              This unit is in rads
     */
    private void setUpperArmAngle(double angle) {

        angle = MathUtil.clamp(
                angle,
                ArmConstants.UPPER_ARM_LOWER_LIMIT,
                ArmConstants.UPPER_ARM_UPPER_LIMIT);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = ArmConstants.upperfeedForward.calculate(angle, 0);

        upperArmPIDController.setFF(
                FF,
                // Toggle between slot 0, 1, or 2
                // If we should use slot 1 and 2, use 2
                // else, use 1
                // else, use 0 (default)
                upperPIDSlot1
                        ? upperPIDSlot2
                                ? 2
                                : 1
                        : 0);

        // Set the position of the neo controlling the upper arm to
        // Toggle between all PID slots
        upperArmPIDController.setReference(
                angle,
                ControlType.kPosition,
                // Toggle between slot 0,1, or 2
                // If we should use slot 1 and 2, use 2
                // else, use 1
                // else, use 0 (default)
                upperPIDSlot1
                        ? upperPIDSlot2
                                ? 2
                                : 1
                        : 0);
    }

    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in rads
     */
    private void setLowerArmAngle(double position) {

        position = MathUtil.clamp(
                position,
                ArmConstants.LOWER_ARM_LOWER_LIMIT,
                ArmConstants.LOWER_ARM_UPPER_LIMIT);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = ArmConstants.lowerfeedForward.calculate(position, 0);
        lowerArmPIDController.setFF(FF);
        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        lowerArmPIDController.setReference(position, ControlType.kPosition);
    }

        /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in rads
     */
    public double getUpperArmAngle() {
        return (!FieldConstants.IS_SIMULATION) ? upperArmEncoder.getPosition() : upperArmEncoderSimPosition;
    }

    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in rads
     */
    public double getLowerArmAngle() {
        return (!FieldConstants.IS_SIMULATION) ? lowerArmEncoder.getPosition() : lowerArmEncoderSimPosition;
    }

    public boolean getAtDesiredPositions() {
        return this.armsAtDesiredPosition;
    }

    public double getXPosition() {
        return this.armXPos;
    }

    public double getYPosition() {
        return this.armYPos;
    }

    // Check if the arm is at its desired position and that position is a placement
    // index,
    // Note that the stowed position is not a placement index, but can be used as
    // hybrid placement
    public boolean getAtPlacementPosition() {
        return (armPosDimension1 == PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX ||
                armPosDimension1 == PlacementConstants.AUTO_CUBE_HIGH_INDEX ||
                armPosDimension1 == PlacementConstants.CONE_HIGH_PLACEMENT_INDEX ||
                armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX ||
                armPosDimension1 == PlacementConstants.CUBE_MID_INDEX ||
                armPosDimension1 == PlacementConstants.AUTO_CUBE_MID_INDEX ||
                armPosDimension1 == PlacementConstants.CONE_MID_PLACEMENT_INDEX ||
                armPosDimension1 == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX ||
                armPosDimension1 == PlacementConstants.HYBRID_PLACEMENT_INDEX) &&
                armsAtDesiredPosition;
    }

    public boolean getAtPrepIndex() {
        return (armPosDimension1 == PlacementConstants.FLOOR_INTAKE_PREP_INDEX ||
                armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_INDEX ||
                armPosDimension1 == PlacementConstants.CONE_MID_PREP_INDEX ||
                followingTrajectory);
    }

    // If we are at a prep index,
    // set the index to the prep_to_place equivelent
    public void finishPlacement() {
        if (getAtPrepIndex() && armsAtDesiredPosition) {
            followingTrajectory = false;
            switch (armPosDimension1) {
                case PlacementConstants.CONE_HIGH_PREP_INDEX:
                    setArmIndex(PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX);
                    break;
                case PlacementConstants.CONE_MID_PREP_INDEX:
                    setArmIndex(PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX);
                    break;
            }
        }
    }

    public Command getAutoStowCommand() {
        return runOnce(() -> {
            if (getArmIndex() == PlacementConstants.CONE_HIGH_PLACEMENT_INDEX ||
            getArmIndex() == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX ||
            getArmIndex() == PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX) 
        {
      
            setArmIndex(PlacementConstants.HIGH_TO_STOWED_INDEX);
            startTrajectory(
                PlacementConstants.CONE_MODE
                    ? PlacementConstants.HIGH_CONE_TO_STOWED_TRAJECTORY
                    : PlacementConstants.HIGH_CUBE_TO_STOWED_TRAJECTORY
            );

        } else if (getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX ||
                getArmIndex() == PlacementConstants.CONE_MID_PLACEMENT_INDEX ||
                getArmIndex() == PlacementConstants.CUBE_MID_INDEX) 
        {

            setArmIndex(PlacementConstants.MID_TO_STOWED_INDEX);

        } else {

            setArmIndex(PlacementConstants.STOWED_INDEX);
        
        }
        });
    }

    public Command finishPlacmentCommand() {
        return runOnce(() -> finishPlacement());
    }

    public Command getUnflipCommand() {
        return runOnce(() -> setArmIndex(PlacementConstants.ARM_FLIP_INDEX));
    }

    public Command getPOVHighCommand() {
        return runOnce(() -> {
            // Hot reload means that we are inputting prep to place -> prep
            boolean hotReloadHigh = getArmIndex() == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX;
            setArmIndex((PlacementConstants.CONE_MODE) ? PlacementConstants.CONE_HIGH_PREP_INDEX : PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX);
            if (!hotReloadHigh) { 
                startTrajectory((PlacementConstants.CONE_MODE) ? PlacementConstants.HIGH_CONE_TRAJECTORY : PlacementConstants.HIGH_CUBE_TRAJECTORY); 
            }
        });
    }

    public Command getPOVDownCommand() {
        return runOnce(() -> {
            setArmIndex((PlacementConstants.CONE_MODE) ? PlacementConstants.CONE_INTAKE_INDEX : PlacementConstants.CUBE_INTAKE_INDEX);
        });
    }

    public Command getPOVLeftCommand(DoubleSupplier xPosition) {
        return runOnce(() -> {
            if (FieldConstants.GAME_MODE == FieldConstants.GameMode.TEST) {
                boolean hotReloadMid = getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX;
                setArmIndex((PlacementConstants.CONE_MODE) ? PlacementConstants.CONE_MID_PREP_INDEX : PlacementConstants.CUBE_MID_INDEX);
                if (!hotReloadMid && PlacementConstants.CONE_MODE) { 
                    startTrajectory(PlacementConstants.MID_CONE_TRAJECTORY); 
                }
            }
            else {
                setArmMidOrHumanPlayer(xPosition.getAsDouble());
            }
        });
    }

    public Command getPOVRightCommand(DoubleSupplier xPosition) {
        return runOnce(() -> {
            if (FieldConstants.GAME_MODE == FieldConstants.GameMode.TEST) {
                setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
            }
            else {
                setArmMidOrHumanPlayer(xPosition.getAsDouble());
            }
        });
    }

    public Command getDriveCommand(IntSupplier index) {
        return runOnce(() -> setArmIndex(index.getAsInt()));
    }

    public void setArmMidOrHumanPlayer(double xPosition) {
        /*
         * If we are on blue alliance, make it so that the left and right buttons set the arm to human tag
         *   when the robot is halfway across the field
         * this code is in place due to our operator confusing the buttons and clicking the wrong one.
         */
        if ((FieldConstants.ALLIANCE == Alliance.Blue) ? (xPosition > FieldConstants.FIELD_WIDTH_METERS/2) : (xPosition < FieldConstants.FIELD_WIDTH_METERS/2)) {
            setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
        }
        else {
            boolean hotReloadMid = getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX;
            setArmIndex((PlacementConstants.CONE_MODE) ? PlacementConstants.CONE_MID_PREP_INDEX : PlacementConstants.CUBE_MID_INDEX);
            if (!hotReloadMid && PlacementConstants.CONE_MODE) { startTrajectory(PlacementConstants.MID_CONE_TRAJECTORY); }
        }
    }

    /**
     * If we are at some placement index,
     * (or intake index)
     * and we change coneMode (to cone or cube mode)
     * set the index to the equivelent index in the new mode
     * to reduce the need to input another button
     */
    public Command handleConeModeChange() {
        return runOnce(() -> {
                // Notice all of the trues here,
                // These make sure that the arm doesn't go back to the transition position
                // which speeds up the process of changing modes
                switch (armPosDimension1) {
                    case PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX:
                        setArmIndex(PlacementConstants.CONE_HIGH_PREP_INDEX, true);
                        break;
                    case PlacementConstants.CUBE_MID_INDEX:
                        setArmIndex(PlacementConstants.CONE_MID_PREP_INDEX, true);
                        break;
                    case PlacementConstants.CONE_HIGH_PREP_INDEX:
                        setArmIndex(PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, true);
                        break;
                    case PlacementConstants.CONE_MID_PREP_INDEX:
                        setArmIndex(PlacementConstants.CUBE_MID_INDEX, true);
                        break;
                    case PlacementConstants.CONE_INTAKE_INDEX:
                        setArmIndex(PlacementConstants.CUBE_INTAKE_INDEX, true);
                        break;
                    case PlacementConstants.CUBE_INTAKE_INDEX:
                        setArmIndex(PlacementConstants.CONE_INTAKE_INDEX, true);
                        break;
                }
            }
        );
    }

    public Command toggleOperatorOverride() {
        return runOnce(() -> this.operatorOverride = !operatorOverride);
    }

    public Command getOperatorOverrideDriveCommand(Supplier<Translation2d> operatorLeftAxis) {
        return run(() -> drive(new Translation2d(operatorLeftAxis.get().getX(), -operatorLeftAxis.get().getY())));
    }

    public boolean getOperatorOverride() {
        return this.operatorOverride;
    }

    // Set the motors to coast mode
    public void setCoastMode() {
        lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    // This exists for the purpose of being able to manually move the upper arm
    // easier.
    public void setUpperArmCoastMode() {
        upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    // Set the motors to brake mode
    public void setBrakeMode() {
        lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    // Very rough code below: this method is meant to take an encoder
    // and zero it with the knowledge that it is at the hard stop
    // when the method is called
    public void zeroLowerArmEncoder() {
        // The constant found here can be found in
        // REV Hardware client when the arm is pointed straight up
        // Then, depending on if the value is more or less than PI,
        // Add or subtract PI to the value

        // Note, that this value must be within the range of 0-2PI
        // absolutePosition = lowerArmEncoder.getPosition() +
        // lowerArmEncoder.getZeroOffset();
        // test that 2024 friends :) ^^ itll be kinda nice to just do
        // lowerArmEncoder.setZeroOffset(absolutePosition + (absolutePosition > Math.PI)
        // ? -Math.PI : Math.PI);

        lowerArmEncoder.setZeroOffset(5.4559006 - Math.PI);
    }

    // Very rough code below: this method is meant to take an encoder
    // and zero it with the knowledge that it is at the hard stop
    // when the method is called
    public void zeroUpperArmEncoder() {
        // The constant found here can be found in
        // REV Hardware client when the arm is pointed straight up
        // Then, depending on if the value is more or less than PI,
        // Add or subtract PI to the value
        upperArmEncoder.setZeroOffset(2.7098798 + Math.PI);
    }

    public void logArmData() {
        SmartDashboard.putData("ArmDesired2D", armDesiredMechanism);
        SmartDashboard.putNumberArray("ArmDesired3D", getPoseData(lowerReferenceAngle, upperReferenceAngle));

        SmartDashboard.putNumberArray("ArmActual3D", getPoseData(getLowerArmAngle(), getUpperArmAngle()));
        SmartDashboard.putNumberArray("ArmREALXY", new double[] { this.armXPos, this.armYPos });
    }

    private double[] getPoseData(double lowerArmAngle, double upperArmAngle) {
        var lowerPose = new Pose3d(
                0.0,
                0.0,
                0.29,
                new Rotation3d(0.0, -lowerArmAngle, 0.0));

        var upperPose = lowerPose.transformBy(
                new Transform3d(
                        new Translation3d(0.0, 0.0, -Units.inchesToMeters(ArmConstants.LOWER_ARM_LENGTH)),
                        new Rotation3d(0.0, -upperArmAngle, 0.0)));

        return Pose3dLogger.composePose3ds(lowerPose, upperPose);
    }
}
