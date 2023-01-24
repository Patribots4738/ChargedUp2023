package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import math.*;
import math.Constants.ArmConstants;

public class Arm implements Loggable {

  /**
   * What the arm positions look like and the index in the array
   *         4
   * O        __     8
   *  1      |      7
   *       3 | 5
   *    2  |||||  6
   */

  int[][] armPos = {{0, 2}, {1, 1}, {2, 0}, {3, 1}, {4, 4}, {5, 1}, {6, 0}, {7, 2}, {8, 3}};
  int armPosIndex = 0;
  
  ArmCalcuations armCalculations = new ArmCalcuations();

  private final CANSparkMax _lowerArm;
  private final CANSparkMax _upperArm;
  
  private final RelativeEncoder _lowerArmEncoder;
  private final RelativeEncoder _upperArmEncoder;

  private final SparkMaxPIDController _lowerArmPIDController;
  private final SparkMaxPIDController _upperArmPIDController;

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


    // Set the idle (brake) mode for the lower and upper SPARK MAX(s)
    _lowerArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    _upperArm.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Save the SPARK MAX configuration. If a SPARK MAX 
    // browns out, it will retain the last configuration
    _lowerArm.burnFlash();
    _upperArm.burnFlash();
  }

  /**
   * Return the current position of both segments
   * 
   * @return the current position of both segments
   */
  public double[] getArmPosition() {
    return new double[] {
      _lowerArmEncoder.getPosition(), _upperArmEncoder.getPosition()
    };
  }

  /**
   * Calculate the position of the arm based on the joystick input
   * as an absolute position in inches, multiplied by 
   * Constants.ArmConstants.kMaxReachX,Y respectivly
   * @param armX the x position of the joystick
   * @param armY the y position of the joystick
   */
  public void drive(double armX, double armY) {

    // Make sure armX and armY are within the range of 0 to 1
    // We cannot reach below the ground sadge
    armY = (armY < 0) ? 0 : armY;

    // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
    // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
    double upperArmAngle = armCalculations.getLowerAngle(armX, armY);
    double lowerArmAngle = armCalculations.getUpperAngle(armX, armY, upperArmAngle);

    // If upperArmAngle is NotANumber, set lowerArmAngle and upperArmAngle to zero,
    // This is because lowerArmAngle is reliant on upperArmAngle
    if (Double.isNaN(upperArmAngle)) {
      lowerArmAngle = 0;
      upperArmAngle = 0;
    }

    // setLowerArmPosition(Units.radiansToRotations(lowerArmAngle));
    setUpperArmPosition(Units.radiansToRotations(upperArmAngle));

  }

  /**
   * Set the position of the upper arm
   *
   * @param position the position to set the upper arm to
   * This unit is in revolutions
   */
  public void setUpperArmPosition (double position) {

    // Do not let the arm go past the limits defined in ArmConstants
    if (position > Units.degreesToRotations(ArmConstants.kUpperFreedom)) {
      position = Units.degreesToRotations(ArmConstants.kUpperFreedom);
    }
    else if (position < -Units.degreesToRotations(ArmConstants.kUpperFreedom)) {
      position = -Units.degreesToRotations(ArmConstants.kUpperFreedom);
    }

    /*
      FF is a predictive formula that uses the input of where we want to be to predict the path required to get there
      This is an open loop, and thus unable to react to the effects of disturbances / unknown disturbances.
      Because of this, we need to give it disturbance data, which is why we use sysID
      It then uses the data to predict the output of the motor, creating a more accurate prediction

      In contrast, a closed loop is commonly used as PID, looking and reacting to the current position
      in opposition to the desired position, taking note of any disturbance that affected the motor.
      Since weight, and more importantly, the things connected to the arm are changing,
      making it difficult to perfectly tune

      We are using a very small combination of both, using the FF to predict the path, and PID to correct for error
      FF will be doing the heavy lifting, and PID will only be small finishing touches for perfection.
      👌
     */
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
    // By multiplying the position by the gear ratio
    double neoPosition = position * ArmConstants.kUpperArmGearRatio;

    // Set the position of the neo controlling the upper arm to
    // the converted position, neoPosition
    _upperArmPIDController.setReference(neoPosition, ControlType.kPosition);

  }

  /**
   * Set the position of the lower arm
   * 
   * @param position the position to set the lower arm to
   * This unit is in full rotations
   */
  public void setLowerArmPosition (double position) {

    // Do not let the arm go past the limits defined in ArmConstants
    if (position > Units.degreesToRotations(ArmConstants.kLowerFreedom)) {
      position = Units.degreesToRotations(ArmConstants.kLowerFreedom);
    }
    else if (position < -Units.degreesToRotations(ArmConstants.kLowerFreedom)) {
      position = -Units.degreesToRotations(ArmConstants.kLowerFreedom);
    }

    /*
      FF is a predictive formula that uses the input of where we want to be to predict the path required to get there
      This is an open loop, and thus unable to react to the effects of disturbances / unknown disturbances.
      Because of this, we need to give it disturbance data, which is why we use sysID
      It then uses the data to predict the output of the motor, creating a more accurate prediction

      In contrast, a closed loop is commonly used as PID, looking and reacting to the current position
      in opposition to the desired position, taking note of any disturbance that affected the motor.
      Since weight, and more importantly, the things connected to the arm are changing,
      making it difficult to perfectly tune

      We are using a very small combination of both, using the FF to predict the path, and PID to correct for error
      FF will be doing the heavy lifting, and PID will only be small finishing touches for perfection.
      👌
     */
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
    double neoPosition = position * ArmConstants.kUpperArmGearRatio;

    // Set the position of the neo controlling the upper arm to
    // the converted position, neoPosition
    _upperArmPIDController.setReference(neoPosition, ControlType.kPosition);
  }

  /**
   * Reset the encoders to zero the arm when initiating the arm
   * Will not be needed in the future because we will have absolute encoders
   */
  public void resetEncoders() {

    _lowerArmEncoder.setPosition(0);
    _upperArmEncoder.setPosition(0);

  }
}