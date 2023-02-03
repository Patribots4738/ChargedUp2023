package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Claw {
    CANSparkMax _clawMotor = new CANSparkMax(0, MotorType.kBrushless);
    RelativeEncoder _clawEncoder = _clawMotor.getEncoder();
    SparkMaxPIDController _clawPIDController = _clawMotor.getPIDController();

    public void init() {
    }

    public void periodic() {
    }

    public void setBrakeMode() {
        _clawMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        _clawMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setSpeed(double speed) {
        _clawPIDController.setReference(speed, ControlType.kVelocity);
    }

}
