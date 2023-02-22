package hardware;

import java.util.ArrayList;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import debug.Debug;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import math.ArmCalculations;
import math.Constants.ArmConstants;
import math.Constants.PlacementConstants;

import java.util.ArrayList;

public class Arm implements Loggable {

    /**
     * What the arm positions look like and the index in the array
     * 4
     * O        __     8
     * 1      |      7
     * 3 | 5
     * 2  |||||  6
     */

    // ceil -- force round up
    int armPosDimention1 = PlacementConstants.STOWED_PLACEMENT_INDEX;
    int armPosDimention2 = 0;
    private boolean startedTransition = false;
    
    private boolean operatorOverride = false;

    private double armXReference = 0;
    private double armYReference = 0;

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

    @Log
    private double lowerDiff = 0;

    @Log
    private double upperDiff = 0;

    @Log
    private double armXPos = 0;

    @Log
    private double armYPos = 0;

    private final CANSparkMax _lowerArmRight;
    private final CANSparkMax _lowerArmLeft;
    private final CANSparkMax _upperArm;

    private final AbsoluteEncoder _lowerArmEncoder;
    private final AbsoluteEncoder _upperArmEncoder;
    private final SparkMaxPIDController _lowerArmPIDController;
    private final SparkMaxPIDController _upperArmPIDController;

    private final ArmCalculations armCalculations;

    /**
     * Constructs a new Arm and configures the encoders and PID controllers.
     */
    public Arm() {

      _lowerArmRight = new CANSparkMax(ArmConstants.LOWER_ARM_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
      _lowerArmLeft = new CANSparkMax(ArmConstants.LOWER_ARM_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
      _upperArm = new CANSparkMax(ArmConstants.UPPER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

      _lowerArmRight.setIdleMode(IdleMode.kBrake);
      _lowerArmLeft.setIdleMode(IdleMode.kBrake);
      _upperArm.setIdleMode(IdleMode.kCoast);

      // Factory reset, so we get the SPARK MAX to a known state before configuring
      // them. This is useful in case a SPARK MAX is swapped out.
      _lowerArmRight.restoreFactoryDefaults();
      _lowerArmLeft.restoreFactoryDefaults();
      _upperArm.restoreFactoryDefaults();

      _lowerArmEncoder = _lowerArmRight.getAbsoluteEncoder(Type.kDutyCycle);
      _upperArmEncoder = _upperArm.getAbsoluteEncoder(Type.kDutyCycle);

      _lowerArmPIDController = _lowerArmRight.getPIDController();
      _upperArmPIDController = _upperArm.getPIDController();
      _lowerArmPIDController.setFeedbackDevice(_lowerArmEncoder);
      _upperArmPIDController.setFeedbackDevice(_upperArmEncoder);

      _lowerArmEncoder.setPositionConversionFactor(ArmConstants.LOWER_ENCODER_POSITION_FACTOR);
      _lowerArmEncoder.setVelocityConversionFactor(ArmConstants.LOWER_ENCODER_VELOCITY_FACTOR);

      _upperArmEncoder.setPositionConversionFactor(ArmConstants.UPPER_ENCODER_POSITION_FACTOR);
      _upperArmEncoder.setVelocityConversionFactor(ArmConstants.UPPER_ENCODER_VELOCITY_FACTOR);

      // _lowerArmPIDController.setPositionPIDWrappingEnabled(true);
      // _lowerArmPIDController.setPositionPIDWrappingMinInput(ArmConstants.LOWER_ENCODER_POSITION_PID_MIN_INPUT);
      // _lowerArmPIDController.setPositionPIDWrappingMaxInput(ArmConstants.LOWER_ENCODER_POSITION_PID_MAX_INPUT);

      // _upperArmPIDController.setPositionPIDWrappingEnabled(true);
      // _upperArmPIDController.setPositionPIDWrappingMinInput(ArmConstants.UPPER_ENCODER_POSITION_PID_MIN_INPUT);
      // _upperArmPIDController.setPositionPIDWrappingMaxInput(ArmConstants.UPPER_ENCODER_POSITION_PID_MAX_INPUT);

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

      // _lowerArmRight.setSmartCurrentLimit(ArmConstants.LOWER_STALL_LIMIT, ArmConstants.LOWER_FREE_LIMIT, ArmConstants.LOWER_MAX_RPM);
      // _lowerArmLeft.setSmartCurrentLimit(ArmConstants.LOWER_STALL_LIMIT, ArmConstants.LOWER_FREE_LIMIT, ArmConstants.LOWER_MAX_RPM);
      // _upperArm.setSmartCurrentLimit(ArmConstants.UPPER_STALL_LIMIT, ArmConstants.UPPER_FREE_LIMIT, ArmConstants.UPPER_MAX_RPM);

      // Save the SPARK MAX configuration. If a SPARK MAX
      // browns out, it will retain the last configuration
      _lowerArmLeft.follow(_lowerArmRight, true);
      _upperArmEncoder.setInverted(true);

      _lowerArmRight.burnFlash();
      _lowerArmLeft.burnFlash();
      _upperArm.burnFlash();

      armCalculations = new ArmCalculations();
    }

    public void toggleOperatorOverride() {
        this.operatorOverride = !operatorOverride;
    }

    public void periodic() {
        if (!getOperatorOverride()) { indexPeriodic(); }
        setLowerArmPosition(lowerReference);
        setUpperArmPosition(upperReference);
        upperDiff = (Units.radiansToDegrees(upperReference) - Units.radiansToDegrees(getUpperArmPosition()));
        lowerDiff = (Units.radiansToDegrees(lowerReference) - Units.radiansToDegrees(getLowerArmPosition()));
        // Use forward kinematics to get the x and y position of the end effector
        armXPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.cos((getLowerArmPosition() - (Math.PI/2)))) + (ArmConstants.UPPER_ARM_LENGTH * Math.cos((getUpperArmPosition() - Math.PI) + (getLowerArmPosition() - (Math.PI/2)))));
        armYPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.sin((getLowerArmPosition() - (Math.PI/2)))) + (ArmConstants.UPPER_ARM_LENGTH * Math.sin((getUpperArmPosition() - Math.PI) + (getLowerArmPosition() - (Math.PI/2)))));
        System.out.println(String.format("Lower Pos %.3f; Upper Pos %.3f", Math.toDegrees(getLowerArmPosition()), Math.toDegrees(getUpperArmPosition())));
    }

    public void indexPeriodic() {

      // // PlacementConstants.ARM_POSITIONS[armPosDimention1][armPosDimention2]
      if (!startedTransition)
      {
        startedTransition = true;
        drive(PlacementConstants.ARM_POSITIONS[armPosDimention1][armPosDimention2]);
        return;
      }

      // armPosDimention1 = MathUtil.clamp(armPosDimention1, 0, PlacementConstants.ARM_POSITIONS.length-1);
      // System.out.println(String.format("Lower Pos %.3f; Upper Position %.3f, Lower Ref %.3f, Upper Ref %.3f", Math.toDegrees(getLowerArmPosition()), Math.toDegrees(getUpperArmPosition()), Math.toDegrees(lowerReference), Math.toDegrees(upperReference)));

      if (Math.abs(getLowerArmPosition() - (lowerReference + ((lowerReference < 0) ? Math.PI*2 : 0))) < ArmConstants.LOWER_ARM_DEADBAND &&
          Math.abs(getUpperArmPosition() - (upperReference + ((upperReference < 0) ? Math.PI*2 : 0))) < ArmConstants.UPPER_ARM_DEADBAND)
      {
        armPosDimention2++;
        // armPosDimention2 = MathUtil.clamp(armPosDimention2, 0, PlacementConstants.ARM_POSITIONS[armPosDimention1].length-1);
        if (armPosDimention1 >= PlacementConstants.ARM_POSITIONS.length ||
        armPosDimention2 >= PlacementConstants.ARM_POSITIONS[armPosDimention1].length)
        {
          return;
        }
        // System.out.println("Switching dim2 from " + (armPosDimention2-1) + " to " + (armPosDimention2) + "\nArrayInfo: " + PlacementConstants.ARM_POSITIONS[armPosDimention1][armPosDimention2]);
        drive(PlacementConstants.ARM_POSITIONS[armPosDimention1][armPosDimention2]);
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
     * Sets arm index from the PlacementConstants.ARM_POSITIONS array
     *
     * @param index the direction to change the arm index by
     */
    public void setArmIndex(int index) {

        index = MathUtil.clamp(index, 0, PlacementConstants.ARM_POSITIONS.length-1);

        armPosDimention1 = index;
        armPosDimention2 = 0;
        startedTransition = false;

    }

    public int getArmIndex() {
        return armPosDimension1;
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
          this.armXReference += (position.getX());
          this.armYReference += (position.getY());
        } else {
          this.armXReference = position.getX();
          this.armYReference = position.getY();
        }

        // Make sure armX and armY are within the range of 0 to infinity
        // Because we cannot reach below the ground.
        // Even though our arm starts 11 inches above the ground,
        // the claw will be 11 inches from the arm end
        armYReference = (armYReference < 0) ? 0 : armYReference;

        Translation2d armPosition = new Translation2d(armXReference, armYReference);

        // Proof: https://www.desmos.com/calculator/ppsa3db9fa
        // If the distance from zero is greater than the max reach, cap it at the max reach
        if (armPosition.getNorm() >= ArmConstants.MAX_REACH) {
          armPosition = armPosition.times((ArmConstants.MAX_REACH-1) / armPosition.getNorm());
        }
        if (armPosition.getNorm() <= ArmConstants.MIN_REACH) {
          armPosition = armPosition.times((ArmConstants.MIN_REACH+1) / armPosition.getNorm());
        }
        
        // if (armPosition.getY() > ArmConstants.MAX_REACH_Y) {
        //     armPosition = new Translation2d(armPosition.getX(), ArmConstants.MAX_REACH_Y);
        // }
        // if (armPosition.getX() > ArmConstants.MAX_REACH_X) {
        //     armPosition = new Translation2d(ArmConstants.MAX_REACH_X, armPosition.getY());
        // }
        // else if (armPosition.getX() < -ArmConstants.MAX_REACH_X) {
        //     armPosition = new Translation2d(-ArmConstants.MAX_REACH_X, armPosition.getY());
        // }
        // System.out.println(armPosition);

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


        setLowerArmReference(lowerArmAngle + (Math.PI/2));
        setUpperArmReference(upperArmAngle + (Math.PI));

    }

    /**
     * Set the position of an arm
     *
     * @param position the position to set the upper arm to
     *                 This unit is in revolutions
     */
    public void setUpperArmPosition(double position) {

        position = MathUtil.clamp(
          position,
          Math.toRadians(20),
          Math.toRadians(270)
        );

        // Description of FF in Constants :D
        ArmFeedforward feedForward = new ArmFeedforward(
                ArmConstants.S_UPPER,
                ArmConstants.G_UPPER,
                ArmConstants.V_UPPER,
                ArmConstants.A_UPPER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate((position), 0);
        _upperArmPIDController.setFF(FF);

        // Set the position of the neo controlling the upper arm to
        _upperArmPIDController.setReference((position), ControlType.kPosition);

        upperRotation = _upperArmEncoder.getPosition();

        upperRotationList.add(upperRotation);
    }

    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in full rotations
     */
    public void setLowerArmPosition(double position) {

        position = MathUtil.clamp(
                position,
                Math.toRadians(90),
                Math.toRadians(270)
        );

        ArmFeedforward feedForward = new ArmFeedforward(
                ArmConstants.S_LOWER,
                ArmConstants.G_LOWER,
                ArmConstants.V_LOWER,
                ArmConstants.A_LOWER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate((position), 0);
        _lowerArmPIDController.setFF(FF);
        // Set the position of the neo controlling the upper arm to
        // the converted position, neoPosition
        _lowerArmPIDController.setReference((position), ControlType.kPosition);

        lowerRotation = _lowerArmEncoder.getPosition();

        lowerRotationList.add(lowerRotation);
    }

    /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in rads
     */
    public double getUpperArmPosition() {
        return _upperArmEncoder.getPosition();
    }

    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in rads
     */
    public double getLowerArmPosition() {
        return _lowerArmEncoder.getPosition();
    }

    public boolean getAtDesiredPositions() {

        // get the desired position of the arms, using the last index in the armPos[armPosDimension1]
        double upperArmAngle = armCalculations.getUpperAngle(
                armPos[armPosDimension1][armPos[armPosDimension1].length - 1].getX(),
                armPos[armPosDimension1][armPos[armPosDimension1].length - 1].getY());

        double lowerArmAngle = armCalculations.getLowerAngle(
                armPos[armPosDimension1][armPos[armPosDimension1].length - 1].getX(),
                armPos[armPosDimension1][armPos[armPosDimension1].length - 1].getY(), upperArmAngle);

        return (Math.abs(Units.radiansToRotations(upperArmAngle) - getUpperArmPosition()) < 0.01) &&
                (Math.abs(Units.radiansToRotations(lowerArmAngle) - getLowerArmPosition()) < 0.01);
    }

    /**
     * Set the motor to coast mode
     */
    public void setCoastMode() {
        _lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setUpperArmCoastMode() {
      _upperArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Set the motor to brake mode
     */
    public void setBrakeMode() {
      _lowerArmLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
      _lowerArmRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
      _upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void printList() {
      System.out.println(upperRotationList);
    }

    public void zeroLowerArmEncoder() {

        double unadjustedAngle = (_lowerArmEncoder.getPosition() - _lowerArmEncoder.getZeroOffset());

        double zeroAngle = (ArmConstants.LOWER_ARM_HARDSTOP_OFFSET) - unadjustedAngle;

        Rotation2d referenceAngle = Rotation2d.fromRadians(zeroAngle);

        _lowerArmEncoder.setZeroOffset(referenceAngle.getRadians());
    }

    public void zeroUpperArmEncoder() {

          double unadjustedAngle = (_upperArmEncoder.getPosition() - _upperArmEncoder.getZeroOffset());

          double zeroAngle = (ArmConstants.UPPER_ARM_HARDSTOP_OFFSET) - unadjustedAngle;

          Rotation2d referenceAngle = Rotation2d.fromRadians(zeroAngle);

          _upperArmEncoder.setZeroOffset(referenceAngle.getRadians());
    }

    public void setUpperPID() {
      _upperArmPIDController.setP(Debug.armP.getDouble(ArmConstants.UPPER_P));
      _upperArmPIDController.setI(Debug.armI.getDouble(ArmConstants.UPPER_I));
      _upperArmPIDController.setD(Debug.armD.getDouble(ArmConstants.UPPER_D));
    }
}
