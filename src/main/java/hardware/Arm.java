package hardware;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
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
    
    private boolean operatorOverride = false;

    private double armXPos = 0;
    private double armYPos = 0;
    
    // The current rotation of the upper arm
    @Log
    private double upperRotation = 0;
    private final ArrayList<Double> upperRotationList = new ArrayList<Double>();

    // The current rotation of the lower arm
    @Log
    private double lowerRotation = 0;
    private final ArrayList<Double> lowerRotationList = new ArrayList<Double>();
    
    // Math.ceil -- force round up
    private int armPosIndex = (int) Math.ceil(armPos.length / 2);

    // The DESIRED rotation of the upper and lower arm(s)
    private double upperReference = 0;
    private double lowerReference = 0;
    
    private final CANSparkMax _lowerArm;
    private final CANSparkMax _upperArm;
    
    private final RelativeEncoder _lowerArmEncoder;
    private final RelativeEncoder _upperArmEncoder;
    
    private final SparkMaxPIDController _lowerArmPIDController;
    private final SparkMaxPIDController _upperArmPIDController;
    
    ArmCalcuations armCalculations = new ArmCalcuations();

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
        _lowerArmEncoder.setPositionConversionFactor(ArmConstants.kLowerEncoderPositionFactor);
        _upperArmEncoder.setPositionConversionFactor(ArmConstants.kUpperEncoderPositionFactor);

        // _lowerArmEncoder = _lowerArm.getAbsoluteEncoder(Type.kDutyCycle);
        // _upperArmEncoder = _upperArm.getAbsoluteEncoder(Type.kDutyCycle);
        _lowerArmPIDController = _lowerArm.getPIDController();
        _upperArmPIDController = _upperArm.getPIDController();
        _lowerArmPIDController.setFeedbackDevice(_lowerArmEncoder);
        _upperArmPIDController.setFeedbackDevice(_upperArmEncoder);

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
        // _upperArm.setSmartCurrentLimit(ArmConstants.kUpperCurrentLimit);

        // Set the idle (brake) mode for the lower and upper SPARK MAX(s)
        _lowerArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Save the SPARK MAX configuration. If a SPARK MAX
        // browns out, it will retain the last configuration
        _lowerArm.burnFlash();
        _upperArm.burnFlash();
    }

    public void toggleOperatorOverride() {
      this.operatorOverride = !operatorOverride;
    }

    /**
     * Reset the encoders to zero the arm when initiating the arm
     * Will not be needed in the future because we will have absolute encoders
     */
    public void resetEncoders() {

        _lowerArmEncoder.setPosition(0);
        _upperArmEncoder.setPosition(0);//-0.5687823825412326);

    }

    public void periodic() {
    
      setLowerArmPosition(this.lowerReference);
      setUpperArmPosition(this.upperReference);

    }

    public boolean getOperatorOverride() {
      return this.operatorOverride;
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
        // If the arm is not at the reference
        // position (accounting for deadband), don't let the arm index change
        if (Math.abs(_lowerArmEncoder.getPosition() - lowerReference) > ArmConstants.kLowerArmDeadband ||
                Math.abs(_upperArmEncoder.getPosition() - upperReference) > ArmConstants.kUpperArmDeadband)
        {
            armIndex = 0;
        }

        armPosIndex += armIndex;

//        System.out.println("Index: " + armPosIndex + ", X: " + armPos[armPosIndex].getX() + ", Y: " + armPos[armPosIndex].getY());

        drive(new Translation2d(armPos[armPosIndex].getX(), armPos[armPosIndex].getY()));

    }

    /**
     * Calculate the position of the arm based on the joystick input
     * as an absolute position in inches, multiplied by
     * Constants.ArmConstants.kMaxReachX,Y respectively
     *
     * @param armX the x position of the joystick
     * @param armY the y position of the joystick
     */
    public void drive(Translation2d position) {

        // If operatorOverride is true, add the joystick input to the current position
        // recall that this value is in inches
        if (operatorOverride) {
            armXPos += position.getX();
            armYPos += position.getY();
        }
        else {
            this.armXPos = position.getX();
            this.armYPos = position.getY();
        }

        Translation2d armPos = new Translation2d(armXPos, armYPos);

        // Proof: https://www.desmos.com/calculator/ppsa3db9fa
        // If the distance from zero is greater than the max reach, cap it at the max reach
        // Give it a one inch cushon
        if (armPos.getDistance(new Translation2d(0,0)) > ArmConstants.kMaxReach) {
            armPos = armPos.times((ArmConstants.kMaxReach) / armPos.getDistance(new Translation2d(0, 0)));
        }

        if (armPos.getY() > ArmConstants.kMaxReachY) {
            armPos = new Translation2d(armPos.getX(), ArmConstants.kMaxReachY);
        }

        // Make sure armX and armY are within the range of 0 to infinity
        // So we cannot reach below the ground
        armYPos = (position.getY() < 0) ? 0 : armYPos;

        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armXPos, armYPos);
        double lowerArmAngle = armCalculations.getLowerAngle(armXPos, armYPos, upperArmAngle);

        // We do this because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
            lowerArmAngle = lowerRotation;
            upperArmAngle = upperRotation;
        }

        setLowerArmReference(Units.radiansToRotations(lowerArmAngle));
        setUpperArmReference(Units.radiansToRotations(upperArmAngle));

        System.out.println("Upper: " + Units.radiansToDegrees(upperArmAngle) +
                " Lower: " + Units.radiansToDegrees(lowerArmAngle));
    }

    /**
     * Set the position of an arm
     *
     * @param position the position to set the upper arm to
     *                 This unit is in revolutions
     */
    private void setUpperArmPosition(double position) {

        position = MathUtil.clamp(
            position, 
            ArmConstants.UPPER_ARM_FREEDOM_DEGREES, 
            -ArmConstants.UPPER_ARM_FREEDOM_DEGREES
        );

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
        double neoPosition = position;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _upperArmPIDController.setReference(neoPosition, ControlType.kPosition);

        double upperArmEncoderPos = _upperArmEncoder.getPosition();
        upperRotation = upperArmEncoderPos;

        upperRotationList.add(upperRotation);
    }

    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in full rotations
     */
    private void setLowerArmPosition(double position) {
        
        position = MathUtil.clamp(
            position, 
            ArmConstants.LOWER_ARM_FREEDOM_DEGREES, 
            -ArmConstants.LOWER_ARM_FREEDOM_DEGREES
        );

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
        double neoPosition = position;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _lowerArmPIDController.setReference(neoPosition, ControlType.kPosition);

        double lowerArmEncoderPosition = _lowerArmEncoder.getPosition();
        lowerRotation = lowerArmEncoderPosition;

        lowerRotationList.add(lowerRotation);
    }

    /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in revolutions
     */
    public double getUpperArmPosition() {
        return _upperArmEncoder.getPosition();
    }

    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in revolutions
     */
    public double getLowerArmPosition() {
        return _lowerArmEncoder.getPosition();
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

    public void printList() {
      System.out.println(upperRotationList);
  }
}
