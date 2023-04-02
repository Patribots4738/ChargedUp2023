package hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import debug.Debug;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import calc.ArmCalculations;
import calc.Constants.ArmConstants;
import calc.Constants.PlacementConstants;

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
    int armPosDimension1 = PlacementConstants.STOWED_INDEX;
    int armPosDimension2 = 1;

    private boolean startedTransition = false;

    private boolean operatorOverride = false;
    private boolean operatorOverrideBeforeFlip = false;
    private boolean armMirroredBeforeFlip = false;
    private Translation2d armBeforeFlip = PlacementConstants.STOWED_POSITION;
    private int indexBeforeFlip = PlacementConstants.STOWED_INDEX;

    private double armXReference = 0;
    private double armYReference = 0;

    // The current rotation of the upper arm
    @Log
    private double upperRotation = 0;

    // The current rotation of the lower arm
    @Log
    private double lowerRotation = 0;

    // The DESIRED rotation of the upper and lower arm(s)
    @Log
    private double upperReferenceAngle = 0;
    @Log
    private double lowerReferenceAngle = 0;

    @Log
    private double lowerDiff = 0;

    @Log
    private double upperDiff = 0;

    private boolean armsAtDesiredPosition = false;
    
    // The blue arm solution found in desmos here:
    // https://www.desmos.com/calculator/fqyyldertp
    private boolean blueArmSolution = false;
    private boolean armMirrored = false;

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

      _lowerArmLeft.setSmartCurrentLimit(ArmConstants.LOWER_FREE_LIMIT);
      _lowerArmRight.setSmartCurrentLimit(ArmConstants.LOWER_FREE_LIMIT);
      _upperArm.setSmartCurrentLimit(ArmConstants.UPPER_FREE_LIMIT);

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

      // Save the SPARK MAX configuration. If a SPARK MAX
      // browns out, it will retain the last configuration
      _lowerArmLeft.follow(_lowerArmRight, true);
      _upperArmEncoder.setInverted(true);

      _lowerArmRight.burnFlash();
      _lowerArmLeft.burnFlash();
      _upperArm.burnFlash();

      armCalculations = new ArmCalculations();
      setBrakeMode();
    }

    public void periodic() {
        if (!operatorOverride) { indexPeriodic();}
        setLowerArmPosition(lowerReferenceAngle);
        setUpperArmAngle(upperReferenceAngle);
        upperDiff = (Units.radiansToDegrees(upperReferenceAngle) - Units.radiansToDegrees(getUpperArmAngle()));
        lowerDiff = (Units.radiansToDegrees(lowerReferenceAngle) - Units.radiansToDegrees(getLowerArmAngle()));
        // Use forward kinematics to get the x and y position of the end effector
        armXPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.cos((getLowerArmAngle() - (Math.PI/2)))) + (ArmConstants.UPPER_ARM_LENGTH * Math.cos((getUpperArmAngle() - Math.PI) + (getLowerArmAngle() - (Math.PI/2)))));
        armYPos = ((ArmConstants.LOWER_ARM_LENGTH * Math.sin((getLowerArmAngle() - (Math.PI/2)))) + (ArmConstants.UPPER_ARM_LENGTH * Math.sin((getUpperArmAngle() - Math.PI) + (getLowerArmAngle() - (Math.PI/2)))));
    }

    public void indexPeriodic() {

      boolean atDesiredCoarse = (
          Math.abs(lowerReferenceAngle - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_COARSE &&
          Math.abs(upperReferenceAngle - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_COARSE);

      boolean atDesiredFine =
          (Math.abs(lowerReferenceAngle - getLowerArmAngle()) < ArmConstants.LOWER_ARM_DEADBAND_FINE &&
          Math.abs(upperReferenceAngle - getUpperArmAngle()) < ArmConstants.UPPER_ARM_DEADBAND_FINE);

      boolean finalDeadband =
          (armPosDimension2 < PlacementConstants.ARM_POSITIONS[armPosDimension1].length-1)
          ? atDesiredCoarse : atDesiredFine;

      // Syntax example: PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]
      if (!startedTransition)
      {
        startedTransition = true;
        drive(PlacementConstants.ARM_POSITIONS[armPosDimension1][armPosDimension2]);
        return;
      }
      
      // Notice that this code is getting the difference in angle between the arms.
      // It might be better to instead use the difference in position, but I'm not sure. - Hamilton
      if (finalDeadband && !armsAtDesiredPosition)
      {
        armPosDimension2++;

        if ((armPosDimension1 == PlacementConstants.SOLUTION_FLIP_INDEX_POSITIVE ||
            armPosDimension1 == PlacementConstants.SOLUTION_FLIP_INDEX_NEGATIVE) && 
            (armPosDimension2 == PlacementConstants.ARM_POSITIONS[armPosDimension1].length-1))
        {
          // Flip the arm solution and go back to where we were
          blueArmSolution = !blueArmSolution;
        }

        // This line isn't strictly necessary, but could be included.
        // armPosDimension2 = MathUtil.clamp(armPosDimension2, 0, PlacementConstants.ARM_POSITIONS[armPosDimension1].length);
        
        if (armPosDimension2 >= PlacementConstants.ARM_POSITIONS[armPosDimension1].length)
        {
          if (armPosDimension1 == PlacementConstants.SOLUTION_FLIP_INDEX_POSITIVE ||
              armPosDimension1 == PlacementConstants.SOLUTION_FLIP_INDEX_NEGATIVE)
          {
            this.armMirrored = this.armMirroredBeforeFlip;
            drive(new Translation2d((armMirrored) ? -armBeforeFlip.getX() : armBeforeFlip.getX(), armBeforeFlip.getY()));
            // Restore the operator override
            this.operatorOverride = this.operatorOverrideBeforeFlip;
            // If we were using index to flip solution...
            if (!operatorOverride) {
              // Set the arm index to where we were before flipping
              setArmIndex(indexBeforeFlip);
            }
          }
          armsAtDesiredPosition = true;
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

        index = MathUtil.clamp(index, 0, PlacementConstants.ARM_POSITIONS.length-1);

        // Check if we are already at the desired index, and if we are not operator overriding,
        // This is because, if we are operator overriding, we want to be able to go to any index
        // If we are trying to go to a floor index, allow it to restart itself
        if ((index == armPosDimension1 || 
              (index == PlacementConstants.STOWED_INDEX && 
              armPosDimension1 == PlacementConstants.HIGH_TO_STOWWED_INDEX)) &&
            index != PlacementConstants.CONE_FLIP_INDEX && 
            index != PlacementConstants.CONE_INTAKE_INDEX && 
            index != PlacementConstants.CUBE_INTAKE_INDEX &&
            !operatorOverride) 
        {
          startedTransition = true;
          return;
        }
        else {
          startedTransition = false;
          armsAtDesiredPosition = false;
        }

        armPosDimension2 = 0;

        // If we are flipping the arm solution, save the current arm state information
        if ((index == PlacementConstants.SOLUTION_FLIP_INDEX_POSITIVE || index == PlacementConstants.SOLUTION_FLIP_INDEX_NEGATIVE))
        {
          this.armBeforeFlip = new Translation2d(armXReference, armYReference);
          this.operatorOverrideBeforeFlip = this.operatorOverride;
          this.indexBeforeFlip = armPosDimension1;
          this.armMirroredBeforeFlip = armMirrored;
          armMirrored = false;
        }
        
        armPosDimension1 = index;
        // Turn off operator override to prevent arm.drive from setting values wrong
        this.operatorOverride = false;

    }

    public int getArmIndex() {
        return this.armPosDimension1;
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
          // If the robot is facing left, have left joystick be positive
          // If the robot is facing right, have left joystick be negative
          this.armXReference += (position.getX());
          this.armYReference += (position.getY());
        } else {
          // If the arm is mirrored, invert all incoming X values
          this.armXReference = position.getX() * (this.armMirrored ? -1 : 1);
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
        // If the arm is trying to reach further than 48" out from the chassis, cap it at 48"
        // The field gives us this limit.
        if (armPosition.getX() > ArmConstants.MAX_REACH_X) {
          armPosition = new Translation2d(ArmConstants.MAX_REACH_X, armPosition.getY());
        }
        else if (armPosition.getX() < -ArmConstants.MAX_REACH_X) {
          armPosition = new Translation2d(-ArmConstants.MAX_REACH_X, armPosition.getY());
        }


        // Check if the arm is in the flip zone for black (but using the blue solution)
        // aka the positive solution
        if (blueArmSolution) {
          if (armPosition.getX() > ArmConstants.ARM_FLIP_X && getArmIndex() != PlacementConstants.SOLUTION_FLIP_INDEX_NEGATIVE) {
            // Set the arm to go to the nearest flip position
            // in this case it is positive, because armPosition.getX() > 0
            setArmIndex(PlacementConstants.SOLUTION_FLIP_INDEX_NEGATIVE);
            return;
          }
        }
        // Or... if the arm is in the flip zone for blue (but using the black solution)
        else if (armPosition.getX() < -ArmConstants.ARM_FLIP_X && getArmIndex() != PlacementConstants.SOLUTION_FLIP_INDEX_POSITIVE) {
          // Set the arm to go to the nearest flip position
          // in this case it is negative, because armPosition.getX() < ARM_FLIP_X
          setArmIndex(PlacementConstants.SOLUTION_FLIP_INDEX_POSITIVE);
          return;
        }


        // Get lowerArmAngle and upperArmAngle, the angles of the lower and upper arm
        // Q2 must be gotten first, because lowerArmAngle is reliant on upperArmAngle
        double upperArmAngle = armCalculations.getUpperAngle(armPosition.getX(), armPosition.getY(), blueArmSolution);
        double lowerArmAngle = armCalculations.getLowerAngle(armPosition.getX(), armPosition.getY(), upperArmAngle);

        // If upperArmAngle is NaN, then tell the arm not to change position
        // We only check upperArmAngle because lowerArmAngle is reliant on upperArmAngle
        if (Double.isNaN(upperArmAngle)) {
          System.out.println("Upper angle NAN " + armPosition + " " + armPosition.getNorm());
          return;
        }

        // These offsets are a result of the arm's "Zero" being straight down
        lowerArmAngle += (Math.PI/2);
        // These offsets are a result of the arm's "Zero" being straight down
        upperArmAngle += Math.PI;

        // Clamp the output angles as to not murder our precious hard stops
        upperArmAngle = MathUtil.clamp(
          upperArmAngle,
          ArmConstants.UPPER_ARM_LOWER_LIMIT,
          ArmConstants.UPPER_ARM_UPPER_LIMIT
        );

        // Clamp the output angles as to not murder our precious hard stops
        lowerArmAngle = MathUtil.clamp(
          lowerArmAngle,
          ArmConstants.LOWER_ARM_LOWER_LIMIT,
          ArmConstants.LOWER_ARM_UPPER_LIMIT
        );

        // Set the reference values to the modified X and Y values
        // This is especially important for the operatorOverride going OOB
        this.armXReference = armPosition.getX();
        this.armYReference = armPosition.getY();

        // Finally, set the reference values for the lower and upper arm:
        // Add PI/2 to lowerArmAngle...
        // because the calculated angle is relative to the ground,
        // And the zero for the encoder is the direction of gravity
        // Add PI to upperArmAngle...
        // because armCalculations gives us the angle relative to the upper arm
        setLowerArmReference(lowerArmAngle);
        setUpperArmReference(upperArmAngle);

    }

    public void setLowerArmReference(double reference) {
      this.lowerReferenceAngle = reference;
    }

    public void setUpperArmReference(double reference) {
      this.upperReferenceAngle = reference;
    }

    /**
     * Set the position of an arm
     *
     * @param angle the position to set the upper arm to
     *                 This unit is in rads
     */
    public void setUpperArmAngle(double angle) {

        angle = MathUtil.clamp(
            angle,
            ArmConstants.UPPER_ARM_LOWER_LIMIT,
            ArmConstants.UPPER_ARM_UPPER_LIMIT
        );

        // Description of FF in Constants :D
        ArmFeedforward feedForward = new ArmFeedforward(
            ArmConstants.S_UPPER,
            ArmConstants.G_UPPER,
            ArmConstants.V_UPPER,
            ArmConstants.A_UPPER);

        // Get the feedforward value for the position,
        // Using a predictive formula with sysID given data of the motor
        double FF = feedForward.calculate((angle), 0);
        _upperArmPIDController.setFF(FF);

        // Set the position of the neo controlling the upper arm to
        _upperArmPIDController.setReference((angle), ControlType.kPosition);

        upperRotation = _upperArmEncoder.getPosition();
    }

    /**
     * Set the position of the lower arm
     *
     * @param position the position to set the lower arm to
     *                 This unit is in rads
     */
    public void setLowerArmPosition(double position) {

        position = MathUtil.clamp(
            position,
            ArmConstants.LOWER_ARM_LOWER_LIMIT,
            ArmConstants.LOWER_ARM_UPPER_LIMIT
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
    }

    /**
     * Get the current position of the upper arm
     *
     * @return the current position of the upper arm
     * This unit is in rads
     */
    public double getUpperArmAngle() {
        return _upperArmEncoder.getPosition();
    }

    /**
     * Get the current position of the lower arm
     *
     * @return the current position of the lower arm
     * This unit is in rads
     */
    public double getLowerArmAngle() {
        return _lowerArmEncoder.getPosition();
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

    // Check if the arm is at its desired position and that position is a placement index,
    // Note that the stowed position is not a placement index, but can be used as hybrid placement
    public boolean getAtPlacementPosition() {
      return (armPosDimension1 == PlacementConstants.CUBE_HIGH_INDEX ||
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

    public boolean getAtFlipPosition() {
      return (armPosDimension1 == PlacementConstants.SOLUTION_FLIP_INDEX_POSITIVE ||
              armPosDimension1 == PlacementConstants.SOLUTION_FLIP_INDEX_NEGATIVE) &&
              armsAtDesiredPosition;
    }

    public boolean getAtPrepIndex() {
      return (armPosDimension1 == PlacementConstants.FLOOR_INTAKE_PREP_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_HIGH_PREP_INDEX ||
              armPosDimension1 == PlacementConstants.CONE_MID_PREP_INDEX);
    }

    public void finishPlacement() {
      if (getAtPrepIndex()) {
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

    public void toggleOperatorOverride() {
      this.operatorOverride = !operatorOverride;
    }

    public boolean getOperatorOverride() {
      return this.operatorOverride;
    }

    public void setArmMirrored(boolean armMirrored) {
      boolean changedState = (this.armMirrored != armMirrored);
      this.armMirrored = armMirrored;
      // Only if we are changing the state of the arm mirrored
      // Then drive the robot to the new reference point
      if (changedState) {
        if (armMirrored) {
          // Since the robot is assumed to be flipped over, 
          // tell the arm to push out hard on the X axis
          // Note that the arm mirroring process will first
          // go through the mirroring indices, so we don't need to worry about
          // the arm jolting to position.
          this.armXReference = 40;
          drive(new Translation2d(armXReference, armYReference));
        }
        else {
          // Drive the arm to just past the flip point to trigger the motion
          this.armXReference = (ArmConstants.ARM_FLIP_X + 0.1);
          drive(new Translation2d(armXReference, armYReference));
        }
      }
    }

    public boolean getArmMirrored() {
      return this.armMirrored;
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

    // Very rough code below: this method is meant to take an encoder
    // and zero it with the knowledge that it is at the hard stop
    // when the method is called
    public void zeroLowerArmEncoder() {

        double unadjustedAngle = (_lowerArmEncoder.getPosition() - _lowerArmEncoder.getZeroOffset());
        double zeroAngle = (ArmConstants.LOWER_ARM_UPPER_LIMIT) - unadjustedAngle;
        Rotation2d referenceAngle = Rotation2d.fromRadians(zeroAngle);
        _lowerArmEncoder.setZeroOffset(referenceAngle.getRadians());
    }

    // Very rough code below: this method is meant to take an encoder
    // and zero it with the knowledge that it is at the hard stop
    // when the method is called
    public void zeroUpperArmEncoder() {

          double unadjustedAngle = (_upperArmEncoder.getPosition() - _upperArmEncoder.getZeroOffset());
          double zeroAngle = (ArmConstants.UPPER_ARM_LOWER_LIMIT) - unadjustedAngle;
          Rotation2d referenceAngle = Rotation2d.fromRadians(zeroAngle);
          _upperArmEncoder.setZeroOffset(referenceAngle.getRadians());
    }

    public void setUpperPID() {
      _upperArmPIDController.setP(Debug.armP.getDouble(ArmConstants.UPPER_P));
      _upperArmPIDController.setI(Debug.armI.getDouble(ArmConstants.UPPER_I));
      _upperArmPIDController.setD(Debug.armD.getDouble(ArmConstants.UPPER_D));
    }
}
