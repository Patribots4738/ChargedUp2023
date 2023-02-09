package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import math.Constants;

public class Claw {
    private final CANSparkMax _claw;
    private final RelativeEncoder _clawEncoder;
    private final SparkMaxPIDController _clawPIDController;
    private double desiredSpeed = 0;

    public Claw() {

        _claw = new CANSparkMax(ClawConstants.kClawMotorCanID, MotorType.kBrushless);
        _claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _claw.restoreFactoryDefaults();

        _clawEncoder = _claw.getEncoder();
        _clawEncoder.setPositionConversionFactor(ClawConstants.kClawPositionConversionFactor);

        _clawPIDController = _claw.getPIDController();
        _clawPIDController.setFeedbackDevice(_clawEncoder);

        _clawPIDController.setP(ClawConstants.kClawP);
        _clawPIDController.setI(ClawConstants.kClawI);
        _clawPIDController.setD(ClawConstants.kClawD);
        _clawPIDController.setFF(ClawConstants.kClawFF);
        _clawPIDController.setOutputRange(
                ClawConstants.kClawMinOutput,
                ClawConstants.kClawMaxOutput);

        _claw.setSmartCurrentLimit(ClawConstants.kClawStallLimit, ClawConstants.kClawFreeLimit);
        _claw.burnFlash();

    }

    public void resetEncoder() {
        _clawEncoder.setPosition(0);
    }

    public void init() {}

    public void periodic() {
        setSpeed(desiredSpeed);
    }

    public void setDesiredSpeed(double speed) {
        this.desiredSpeed = speed;
    }

    private void setSpeed(double speed) {
        _clawPIDController.setReference(speed, ControlType.kVelocity);
    }

    public void setBrakeMode() {
        _claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        _claw.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

}
