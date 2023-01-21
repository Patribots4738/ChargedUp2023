package hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import math.*;
import math.Constants.ArmConstants;

public class Arm {

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
  
  private final AbsoluteEncoder _lowerArmEncoder;
  private final AbsoluteEncoder _upperArmEncoder;

  private final SparkMaxPIDController _lowerArmPIDController;
  private final SparkMaxPIDController _upperArmPIDController;
  
  /**
   * Constructs a new Arm and configures the encoders and PID controllers.
   */
  public Arm() {

    _lowerArm = new CANSparkMax(ArmConstants.kLowerArmMotorCanId, MotorType.kBrushless);
    _upperArm = new CANSparkMax(ArmConstants.kUpperArmMotorCanId, MotorType.kBrushless);
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    _lowerArm.restoreFactoryDefaults();
    _upperArm.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the lower and upper SPARK MAX(s)
    _lowerArmEncoder = _lowerArm.getAbsoluteEncoder(Type.kDutyCycle);
    _upperArmEncoder = _upperArm.getAbsoluteEncoder(Type.kDutyCycle);
    _lowerArmPIDController = _lowerArm.getPIDController();
    _upperArmPIDController = _upperArm.getPIDController();
    _lowerArmPIDController.setFeedbackDevice(_lowerArmEncoder);
    _upperArmPIDController.setFeedbackDevice(_upperArmEncoder);


    // Note that MAXSwerveModule sets the position and velocity "factors"
    // But as of 1/20/2023 I don't know what these are


    // Set PID constants for the lower and upper SPARK MAX(s)
    _lowerArmPIDController.setP(ArmConstants.kLowerP);
    _lowerArmPIDController.setI(ArmConstants.kLowerI);
    _lowerArmPIDController.setD(ArmConstants.kLowerD);
    _lowerArmPIDController.setFF(ArmConstants.kLowerFF);
    _lowerArmPIDController.setOutputRange(ArmConstants.kLowerMinOutput,
        ArmConstants.kLowerMaxOutput);

    _upperArmPIDController.setP(ArmConstants.kUpperP);
    _upperArmPIDController.setI(ArmConstants.kUpperI);
    _upperArmPIDController.setD(ArmConstants.kUpperD);
    _upperArmPIDController.setFF(ArmConstants.kUpperFF);


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
   * Set the position of the lower arm, in radians
   * 
   * @param angle the angle to set the lower arm to
   */
  public void setLowerArmPosition(double angle) {
    _lowerArmPIDController.setReference(angle, ControlType.kPosition);
  }

  /**
   * Set the position of the upper arm, in radians
   * 
   * @param angle the angle to set the upper arm to
   */
  public void setUpperArmPosition(double angle) {
    _upperArmPIDController.setReference(angle, ControlType.kPosition);
  }

  /**
   * Calculate the position of the arm based on the joystick input
   * as an absolute position in inches, multiplied by 
   * Constants.ArmConstants.kMaxReachX,Y respectivly
   * @param armX the x position of the joystick
   * @param armY the y position of the joystick
   */
  public void drive(double armX, double armY) {

      double q2 = armCalculations.getQ2(armX, armY);
      double q1 = armCalculations.getQ1(armX, armY, q2);

      setLowerArmPosition(q1);
      setUpperArmPosition(q2);
  }
    
}