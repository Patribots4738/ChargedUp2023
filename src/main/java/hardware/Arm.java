package hardware;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import math.*;
import math.Constants.ArmConstants;

public class Arm implements Loggable {

    /**
     * What the arm positions look like and the index in the array
     *          4
     * O        __     8
     *  1      |      7
 *          3 | 5
     *   2  |||||  6
     */


    // All armPos values are in inches
    Translation2d[] armPos = {
            // new Translation2d(-40, 28),
            new Translation2d(-36, 36),
            new Translation2d(-10, 13),
            new Translation2d(0, ArmConstants.kMaxReach),
            new Translation2d(10, 13),
            new Translation2d(36, 36),
            // new Translation2d(40, 28),
    };
    int armPosIndex = (int) Math.ceil(armPos.length / 2);

    private double lowerReference = 0;
    private double upperReference = 0;

    ArmCalcuations armCalculations = new ArmCalcuations();

    private final CANSparkMax _lowerArm;
    private final CANSparkMax _upperArm;

    private final RelativeEncoder _lowerArmEncoder;
    private final RelativeEncoder _upperArmEncoder;

    private final SparkMaxPIDController _lowerArmPIDController;
    private final SparkMaxPIDController _upperArmPIDController;

    @Log
    private double upperPos = 0;

    private ArrayList<Double> upperPosList = new ArrayList<Double>();

    @Log
    private double lowerPos = 0;

    private ArrayList<Double> lowerPosList = new ArrayList<Double>();

    /**
     * Constructs a new Arm and configures the encoders and PID controllers.
     */
    public Arm() {

        _lowerArm = new CANSparkMax(ArmConstants.kLowerArmMotorCanId, MotorType.kBrushless);
        _upperArm = new CANSparkMax(ArmConstants.kUpperArmMotorCanId, MotorType.kBrushless);

        _lowerArm.setIdleMode(IdleMode.kBrake);
        _upperArm.setIdleMode(IdleMode.kBrake);

        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        _lowerArm.restoreFactoryDefaults();
        _upperArm.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the lower and upper SPARK MAX(s)
        _lowerArmEncoder = _lowerArm.getEncoder();
        _upperArmEncoder = _upperArm.getEncoder();

        // _lowerArmEncoder = _lowerArm.getAbsoluteEncoder(Type.kDutyCycle);
        // _upperArmEncoder = _upperArm.getAbsoluteEncoder(Type.kDutyCycle);
        _lowerArmPIDController = _lowerArm.getPIDController();
        _upperArmPIDController = _upperArm.getPIDController();
        _lowerArmPIDController.setFeedbackDevice(_lowerArmEncoder);
        _upperArmPIDController.setFeedbackDevice(_upperArmEncoder);


        // Note that MAXSwerveModule sets the position and velocity "factors"
        // But as of 1/20/2023 I don't know what these are

        // as of 1/22/2023, It might be useful to touch because
        // it could reduce the need to multiply position by gear ratio
        // _lowerArmEncoder.setPositionConversionFactor(ArmConstants.kLowerArmGearRatio);

        // Set PID constants for the lower and upper SPARK MAX(s)
        _lowerArmPIDController.setP(ArmConstants.kLowerP);
        _lowerArmPIDController.setI(ArmConstants.kLowerI);
        _lowerArmPIDController.setD(ArmConstants.kLowerD);
        _lowerArmPIDController.setFF(ArmConstants.kLowerFF);
        _lowerArmPIDController.setOutputRange(
                ArmConstants.kLowerMinOutput,
                ArmConstants.kLowerMaxOutput);

        _upperArmPIDController.setP(ArmConstants.kUpperP);
        _upperArmPIDController.setI(ArmConstants.kUpperI);
        _upperArmPIDController.setD(ArmConstants.kUpperD);
        _upperArmPIDController.setFF(ArmConstants.kUpperFF);
        _upperArmPIDController.setOutputRange(
                ArmConstants.kUpperMinOutput,
                ArmConstants.kUpperMaxOutput);

        _lowerArm.setSmartCurrentLimit(ArmConstants.kLowerCurrentLimit);
        _upperArm.setSmartCurrentLimit(ArmConstants.kUpperCurrentLimit);

        // Set the idle (brake) mode for the lower and upper SPARK MAX(s)
        _lowerArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Save the SPARK MAX configuration. If a SPARK MAX
        // browns out, it will retain the last configuration
        _lowerArm.burnFlash();
        _upperArm.burnFlash();
    }

    /**
     * Reset the encoders to zero the arm when initiating the arm
     * Will not be needed in the future because we will have absolute encoders
     */
    public void resetEncoders() {

        _lowerArmEncoder.setPosition(0);
        _upperArmEncoder.setPosition(0);

    }

    public void armPeriodic() {

        setLowerArmPosition(this.lowerReference);
        setUpperArmPosition(this.upperReference);

    }

    public void setLowerArmReference(double reference) {
        this.lowerReference = reference;
    }

    public void setUpperArmReference(double reference) {
        this.upperReference = reference;
    }


    /**
     * Sets arm index from the armPos array
     *
     * @param armIndex the arm index
     */
    public void setArmIndex(int armIndex) {

        armIndex = (armIndex > 0) ? 1 : -1;

        // Make sure armPosIndex is within the range of 0 to armPos.length - 1
        // To prevent out of bounds errors
        if (armPosIndex == 0 && armIndex == -1){
            armIndex = 0;
        }
        if (armPosIndex >= armPos.length - 1 && armIndex == 1) {
            armIndex = 0;
        }

        if (Math.abs(_lowerArmEncoder.getPosition() - lowerReference) > ArmConstants.kLowerArmDeadband ||
                Math.abs(_upperArmEncoder.getPosition() - upperReference) > ArmConstants.kUpperArmDeadband)
        {
            armIndex = 0;
        }

        armPosIndex += armIndex;

//        System.out.println("Index: " + armPosIndex + ", X: " + armPos[armPosIndex].getX() + ", Y: " + armPos[armPosIndex].getY());

        drive(
                armPos[armPosIndex].getX() / ArmConstants.kMaxReach,
                armPos[armPosIndex].getY() / ArmConstants.kMaxReach);
    }

    /**
     * Calculate the position of the arm based on the joystick input
     * as an absolute position in inches, multiplied by
     * Constants.ArmConstants.kMaxReachX,Y respectively
     *
     * @param armX the x position of the joystick
     * @param armY the y position of the joystick
     */
    public void drive(double armX, double armY) {

        // Make sure armX and armY are within the range of 0 to infinity
        // So we cannot reach below the ground
        armY = (armY < 0) ? 0 : armY;

        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armX, armY);
        double lowerArmAngle = armCalculations.getLowerAngle(armX, armY, upperArmAngle);

        // We do this because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
            lowerArmAngle = 0;
            upperArmAngle = 0;
        }

        setLowerArmReference(Units.radiansToRotations(lowerArmAngle));
        setUpperArmReference(Units.radiansToRotations(upperArmAngle));
        System.out.println("Upper: " + Units.radiansToDegrees(upperArmAngle) +
                " Lower: " + Units.radiansToDegrees(lowerArmAngle));
    }


    /**
     * Set the position of the upper arm
     *
     * @param position the position to set the upper arm to
     *                 This unit is in revolutions
     */
    private void setUpperArmPosition(double position) {

        position = setLimits(position, ArmConstants.kUpperFreedom);

        // Description of FF in Constants :D
        ArmFeedforward feedForward = new ArmFeedforward(
                ArmConstants.kSUpper,
                ArmConstants.kGUpper,
                ArmConstants.kVUpper,
                ArmConstants.kAUpper);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate(position, 0);
        _upperArmPIDController.setFF(FF);

        // Calculate the rotations needed to get to the position
        double neoPosition = position * ArmConstants.kUpperArmGearRatio;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _upperArmPIDController.setReference(neoPosition, ControlType.kPosition);

        double upperArmEncoderPos = _upperArmEncoder.getPosition();
        upperPos = upperArmEncoderPos / ArmConstants.kUpperArmGearRatio;

        upperPosList.add(upperPos);
    }


    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in full rotations
     */
    private void setLowerArmPosition(double position) {
        position = setLimits(position, ArmConstants.kLowerFreedom);

        ArmFeedforward feedForward = new ArmFeedforward(
                ArmConstants.kSLower,
                ArmConstants.kGLower,
                ArmConstants.kVLower,
                ArmConstants.kALower);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate(position, 0);
        _lowerArmPIDController.setFF(FF);

        // Calculate the rotations needed to get to the position
        // By multiplying the position by the gear ratio
        double neoPosition = position * ArmConstants.kLowerArmGearRatio;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _lowerArmPIDController.setReference(neoPosition, ControlType.kPosition);

        double lowerArmEncoderPosition = _lowerArmEncoder.getPosition();
        lowerPos = lowerArmEncoderPosition / ArmConstants.kLowerArmGearRatio;

        lowerPosList.add(lowerPos);
    }


    /**
     * Set the limits of the upper arm
     *
     * @param position the position to set the upper arm to
     * @param freedom  the freedom of the upper/lower arm in degrees
     * @return the position, but limited
     */
    private double setLimits(double position, double freedom) {
        if (position > Units.degreesToRotations(freedom)) {
            position = Units.degreesToRotations(freedom);
        } else if (position < -Units.degreesToRotations(freedom)) {
            position = -Units.degreesToRotations(freedom);
        }
        return position;
    }


    /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in revolutions
     */
    public double getUpperArmPosition() {
        return _upperArmEncoder.getPosition() / ArmConstants.kUpperArmGearRatio;
    }


    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in revolutions
     */
    public double getLowerArmPosition() {
        return _lowerArmEncoder.getPosition() / ArmConstants.kLowerArmGearRatio;
    }

    public void printList() {
        System.out.println(upperPosList);
    }

    /**
     * Set the motor to coast mode
     */
    public void setCoastMode() {
        _lowerArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Set the motor to brake mode
     */
    public void setBrakeMode() {
        _lowerArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

}