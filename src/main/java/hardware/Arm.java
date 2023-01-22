package hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

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

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(5, 5);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  
  @Log
  private double _lowerArmPosition;
  @Log
  private double rotations;
  @Log
  private double neoRotations;
  @Log
  private double currentPosition;
  @Log
  double currentVelocity;
  @Log
  ArmFeedforward feedForward;
  @Log
  double FF;
  /**
   * Constructs a new Arm and configures the encoders and PID controllers.
   */
  public Arm() {

    _lowerArm = new CANSparkMax(ArmConstants.kLowerArmMotorCanId, MotorType.kBrushless);
    _upperArm = new CANSparkMax(ArmConstants.kUpperArmMotorCanId, MotorType.kBrushless);

    _lowerArm.setIdleMode(IdleMode.kBrake);
    _upperArm.setIdleMode(IdleMode.kBrake);
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
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
    // _lowerArmEncoder.setPositionConversionFactor(ArmConstants.kLowerArmGearRatio);


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
   * @param angle the angle to set the lower arm to (radians)
   */
  public void setLowerArmPosition(double angle) {

    // Limit the angle from +- pi/3
    if (angle > (Math.PI / 3)) {

      angle = (Math.PI / 3);

    } else if (angle < -(Math.PI / 3)) {

      angle = -(Math.PI / 3);

    }

    rotations = (angle) / (2 * Math.PI);

    neoRotations = 5;// rotations;// * ArmConstants.kLowerArmGearRatio;

    m_goal = new TrapezoidProfile.State(neoRotations, 0);

    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    m_setpoint = profile.calculate(0.02);
    
    feedForward = new ArmFeedforward(
      ArmConstants.kSLower, 
      ArmConstants.kGLower, 
      ArmConstants.kVLower, 
      ArmConstants.kALower);

    FF = feedForward.calculate(m_setpoint.position /*- _lowerArmEncoder.getPosition()*/, m_setpoint.velocity);

    // if (Math.abs(FF) < 0.4) {

    //   FF = 0;

    // }

    //if (Math.abs(m_setpoint.position - _lowerArmEncoder.getPosition()) > 0.25) {

      _lowerArmPIDController.setReference(m_setpoint.position, ControlType.kPosition, 0, FF);

    //} else {

      //_lowerArmPIDController.setReference(0, ControlType.kVoltage);

    //}

   

    /**
     * Turn the angle into rotations of the arm
     * Then, turn rotations in the arm into rotations of the motor
     * Command the motor to rotation the amount of rotations calculated
     */
    /*
    rotations = (angle) / (2 * Math.PI);
    
    neoRotations = rotations * ArmConstants.kLowerArmGearRatio;
    
    // Get the current position, in radians. This is in refrence to the 
    // Arm, so divide by the gear ratio
    currentPosition = (angle - (_lowerArmEncoder.getPosition() * Math.PI * 2)) / ArmConstants.kLowerArmGearRatio;
    
    // Get the current velocity, because getVelocity() returns RPM, turn it to radians
    // and divide by 60s and the gear ratio
    currentVelocity = (_lowerArmEncoder.getVelocity() * Math.PI * 2) / 60 / ArmConstants.kLowerArmGearRatio;
    
    feedForward = new ArmFeedforward(
      ArmConstants.kSLower, 
      ArmConstants.kGLower, 
      ArmConstants.kVLower, 
      ArmConstants.kALower);

    FF = feedForward.calculate(currentPosition, currentVelocity);

    _lowerArmPIDController.setFF(FF);

    System.out.println(FF);

    _lowerArmPIDController.setReference(neoRotations, ControlType.kPosition);
    */

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
      // setUpperArmPosition(q2);


      // ((0.39694) * cos(pos)) + ((0.20953) * sgn(angularVelocity)) + ((0.27319) * (angularVelocity)) + ((0.47396) + (angularAccel))
  }

  public void resetEncoders() {

    _lowerArmEncoder.setPosition(0);
    _upperArmEncoder.setPosition(0);

  }
    
  public void setLowerArmPositionNumber2 (double angle) {

    feedForward = new ArmFeedforward(
      ArmConstants.kSLower, 
      ArmConstants.kGLower, 
      ArmConstants.kVLower, 
      ArmConstants.kALower);

    FF = feedForward.calculate(angle, 0);

    _lowerArmPIDController.setFF(FF);

    _lowerArmPIDController.setReference(angle, ControlType.kPosition);
  }
}