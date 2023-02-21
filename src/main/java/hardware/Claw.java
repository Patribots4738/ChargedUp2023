package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import math.Constants.ClawConstants;

public class Claw {
    private final CANSparkMax _claw;
    private final RelativeEncoder _clawEncoder;
    private double desiredSpeed = 0;

    public Claw() {

        _claw = new CANSparkMax(ClawConstants.CLAW_CAN_ID, MotorType.kBrushless);
        _claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _claw.restoreFactoryDefaults();

        _clawEncoder = _claw.getEncoder();
        _clawEncoder.setPositionConversionFactor(ClawConstants.CLAW_POSITION_CONVERSION_FACTOR);

        _claw.setSmartCurrentLimit(ClawConstants.CLAW_STALL_LIMIT, ClawConstants.CLAW_FREE_LIMIT);
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
        _claw.set(speed);
    }

    public void setBrakeMode() {
        _claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        _claw.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

}
