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
    Translation2d[][] armPos = {
    {
        new Translation2d(-20, 30),
        new Translation2d(-36, 23),
    },
    {
        new Translation2d(0, ArmConstants.MAX_REACH)
    },
    {
        new Translation2d(-12, 19),
        new Translation2d(-28, 13),
        new Translation2d(-32, 10)
    }
    };
    
    // ceil -- force round up
    int armPosDimention1 = (int) Math.ceil(armPos.length / 2);
    int armPosDimention2 = 0;
    
    private boolean operatorOverride = false;

    private double armXPos = 0;
    private double armYPos = 0;

    // The current rotation of the upper arm
    @Log
    private double upperRotation = 0;
    private final ArrayList<Double> upperRotationList = new ArrayList<>();

    // The current rotation of the lower arm
    @Log
    private double lowerRotation = 0;
    private final ArrayList<Double> lowerRotationList = new ArrayList<>();

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

        _lowerArm = new CANSparkMax(ArmConstants.LOWER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
        _upperArm = new CANSparkMax(ArmConstants.UPPER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        _lowerArm.setIdleMode(IdleMode.kBrake);
        _upperArm.setIdleMode(IdleMode.kBrake);

        // Factory reset, so we get the SPARK MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        _lowerArm.restoreFactoryDefaults();
        _upperArm.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the lower and upper SPARK MAX(s)
        _lowerArmEncoder = _lowerArm.getEncoder();
        _upperArmEncoder = _upperArm.getEncoder();
        _lowerArmEncoder.setPositionConversionFactor(ArmConstants.LOWER_ENCODER_POSITION_FACTOR);
        _upperArmEncoder.setPositionConversionFactor(ArmConstants.UPPER_ENCODER_POSITION_FACTOR);

        // _lowerArmEncoder = _lowerArm.getAbsoluteEncoder(Type.kDutyCycle);
        // _upperArmEncoder = _upperArm.getAbsoluteEncoder(Type.kDutyCycle);
        _lowerArmPIDController = _lowerArm.getPIDController();
        _upperArmPIDController = _upperArm.getPIDController();
        _lowerArmPIDController.setFeedbackDevice(_lowerArmEncoder);
        _upperArmPIDController.setFeedbackDevice(_upperArmEncoder);

        // Set PID constants for the lower and upper SPARK MAX(s)
        _lowerArmPIDController.setP(ArmConstants.LOWER_P);
        _lowerArmPIDController.setI(ArmConstants.LOWER_I);
        _lowerArmPIDController.setD(ArmConstants.LOWER_D);
        _lowerArmPIDController.setFF(ArmConstants.LOWER_FF);
        _lowerArmPIDController.setOutputRange(
                ArmConstants.LOWER_MIN_OUTPUT,
                ArmConstants.LOWER_MAX_OUTPUT);

        _upperArmPIDController.setP(ArmConstants.UPPER_P);
        _upperArmPIDController.setI(ArmConstants.UPPER_I);
        _upperArmPIDController.setD(ArmConstants.UPPER_D);
        _upperArmPIDController.setFF(ArmConstants.UPPER_FF);
        _upperArmPIDController.setOutputRange(
                ArmConstants.UPPER_MIN_OUTPUT,
                ArmConstants.UPPER_MAX_OUTPUT);

        _lowerArm.setSmartCurrentLimit(ArmConstants.LOWER_CURRENT_LIMIT);
        // _upperArm.setSmartCurrentLimit(ArmConstants.UPPER_CURRENT_LIMIT);

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
        indexPeriodic();
    }

    public void indexPeriodic() {

        // armPos[armPosDimention1][armPosDimention2]
        if (armPosDimention1 == armPos.length ||
            armPosDimention2 == armPos[armPosDimention1].length) 
        {
            return;
        }

        if (Math.abs(getLowerArmPosition() - lowerReference) > ArmConstants.LOWER_ARM_DEADBAND ||
            Math.abs(getUpperArmPosition() - upperReference) > ArmConstants.UPPER_ARM_DEADBAND)
        {
            armPosDimention2++;
            drive(armPos[armPosDimention1][armPosDimention2]);
        }
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
     * @param index the direction to change the arm index by
     */
    public void setArmIndex(int index) {

        index = MathUtil.clamp(index, 0, armPos.length-1);
        
        armPosDimention1 += index;
        armPosDimention2 = 0;

    }

    public int getArmIndex() {
        return armPosDimention1;
    }

    /**
     * Calculate the position of the arm based on the joystick input
     * as an absolute position in inches, multiplied by
     * Constants.ArmConstants.kMaxReachX,Y respectively
     *
     * @param position either the joystick input or the desired absolute position
     *                 this case is handled under OperatorOverride
     */
    public void drive(Translation2d position) {

        // If operatorOverride is true, add the joystick input to the current position
        // recall that this value is in inches
        if (operatorOverride) {
            this.armXPos += position.getX();
            this.armYPos += position.getY();
        }
        else {
            this.armXPos = position.getX();
            this.armYPos = position.getY();
        }

        // Make sure armX and armY are within the range of 0 to infinity
        // Because we cannot reach below the ground.
        // Even though our arm starts 11 inches above the ground,
        // the claw will be 11 inches from the arm end
        armYPos = (position.getY() < 0) ? 0 : armYPos;

        Translation2d armPos = new Translation2d(armXPos, armYPos);

        // Proof: https://www.desmos.com/calculator/ppsa3db9fa
        // If the distance from zero is greater than the max reach, cap it at the max reach
        // Give it a one-inch cushion
        if (armPos.getDistance(new Translation2d(0,0)) > ArmConstants.MAX_REACH) {
            armPos = armPos.times((ArmConstants.MAX_REACH) / armPos.getDistance(new Translation2d(0, 0)));
        }

        if (armPos.getY() > ArmConstants.MAX_REACH_Y) {
            armPos = new Translation2d(armPos.getX(), ArmConstants.MAX_REACH_Y);
        }

        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armPos.getX(), armPos.getY());
        double lowerArmAngle = armCalculations.getLowerAngle(armPos.getX(), armPos.getY(), upperArmAngle);

        // If upperArmAngle is NaN, then tell the arm not to change position
        // We only check upperArmAngle because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
            return;
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
                ArmConstants.S_UPPER,
                ArmConstants.G_UPPER,
                ArmConstants.V_UPPER,
                ArmConstants.A_UPPER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate(position, 0);
        _upperArmPIDController.setFF(FF);

        // Calculate the rotations needed to get to the position
        double neoPosition = position;

        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _upperArmPIDController.setReference(neoPosition, ControlType.kPosition);

        upperRotation = _upperArmEncoder.getPosition();;

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
                ArmConstants.S_LOWER,
                ArmConstants.G_LOWER,
                ArmConstants.V_LOWER,
                ArmConstants.A_LOWER);

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

        lowerRotation = _lowerArmEncoder.getPosition();

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
