// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package hardware;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import calc.Constants.ModuleConstants;

public class MAXSwerveModule {
    private final CANSparkMax _drivingSparkMax;
    private final CANSparkMax _turningSparkMax;

    private final RelativeEncoder _drivingEncoder;
    private final AbsoluteEncoder _turningEncoder;

    private final SparkMaxPIDController _drivingPIDController;
    private final SparkMaxPIDController _turningPIDController;

    private double _chassisAngularOffset = 0;
    private SwerveModuleState _desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        _drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        _turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        _drivingSparkMax.restoreFactoryDefaults();
        _turningSparkMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        _drivingEncoder = _drivingSparkMax.getEncoder();
        _turningEncoder = _turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        _drivingPIDController = _drivingSparkMax.getPIDController();
        _turningPIDController = _turningSparkMax.getPIDController();
        _drivingPIDController.setFeedbackDevice(_drivingEncoder);
        _turningPIDController.setFeedbackDevice(_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        _drivingEncoder.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        _drivingEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        _turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        _turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        _turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        _turningPIDController.setPositionPIDWrappingEnabled(true);
        _turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
        _turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        _drivingPIDController.setP(ModuleConstants.DRIVING_P);
        _drivingPIDController.setI(ModuleConstants.DRIVING_I);
        _drivingPIDController.setD(ModuleConstants.DRIVING_D);
        _drivingPIDController.setFF(ModuleConstants.DRIVING_FF);
        _drivingPIDController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT,
                ModuleConstants.DRIVING_MAX_OUTPUT);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        _turningPIDController.setP(ModuleConstants.TURNING_P);
        _turningPIDController.setI(ModuleConstants.TURNING_I);
        _turningPIDController.setD(ModuleConstants.TURNING_D);
        _turningPIDController.setFF(ModuleConstants.TURNING_FF);
        _turningPIDController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT,
                ModuleConstants.TURNING_MAX_OUTPUT);

        _drivingSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _turningSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
        _turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        _drivingSparkMax.burnFlash();
        _turningSparkMax.burnFlash();

        _chassisAngularOffset = chassisAngularOffset;
        _desiredState.angle = new Rotation2d(_turningEncoder.getPosition());
        _drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(_drivingEncoder.getVelocity(),
                new Rotation2d(_turningEncoder.getPosition() - _chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                _drivingEncoder.getPosition(),
                new Rotation2d(_turningEncoder.getPosition() - _chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(_turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        _drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        _turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        _desiredState = desiredState;
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        _drivingEncoder.setPosition(0);
    }

    /**
     * Set the motor to coast mode
     */
    public void setCoastMode() {
        _drivingSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _turningSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Set the motor to brake mode
     */
    public void setBrakeMode() {
        _drivingSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _turningSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
}
