package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
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
   * Calculate the position of the arm based on the joystick input
   * as an absolute position in inches, multiplied by 
   * Constants.ArmConstants.kMaxReachX,Y respectivly
   * @param armX the x position of the joystick
   * @param armY the y position of the joystick
   */
  public void drive(double armX, double armY) {

    armY = (armY < 0) ? 0 : armY;

    double q2 = armCalculations.getQ2(armX, armY);
    double q1 = armCalculations.getQ1(armX, armY, q2);

    // If q2 is NaN set q1 and q2 to zero
    if (Double.isNaN(q2)) {
      q1 = 0;
      q2 = 0;
    }
    setLowerArmPosition(Units.radiansToRotations(q1));
    // setUpperArmPosition(q2);
    
  }

  public void resetEncoders() {

    _lowerArmEncoder.setPosition(0);
    _upperArmEncoder.setPosition(0);

  }

  /**
   * Set the position of the lower arm
   * 
   * @param position the position to set the lower arm to
   * This unit is in full rotations
   */
  public void setLowerArmPosition (double position) {

    // Do not let the arm go past 0.2 rotations 
    // aka 72 degrees in both directions
    if (position > 0.2)
    {
      position = 0.2;
    }
    else if (position < -0.2) 
    {
      position = -0.2;
    }

    feedForward = new ArmFeedforward(
      ArmConstants.kSLower, 
      0,//ArmConstants.kGLower, // YO TURN OFF GRAVITY 
      ArmConstants.kVLower, 
      ArmConstants.kALower);

    FF = feedForward.calculate(position, 0);

    _lowerArmPIDController.setFF(FF);

    _lowerArmPIDController.setReference(position*ArmConstants.kLowerArmGearRatio, ControlType.kPosition);
  }
}