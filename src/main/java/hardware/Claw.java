package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import calc.Constants.ClawConstants;
import calc.Constants.PlacementConstants;

public class Claw {

    private final CANSparkMax _claw;
    private final RelativeEncoder _clawEncoder;
    private double desiredSpeed = 0;
    private boolean intakeMode = false;

    // Timer values to have the claw auto outtake for X seconds
    private boolean startedOuttakingBool = false;
    private boolean finishedOuttaking = false;
    private double outtakeSeconds = 0;
    private double startedOuttakingTimestamp = 0;

    public Claw() {

        _claw = new CANSparkMax(ClawConstants.CLAW_CAN_ID, MotorType.kBrushless);
        _claw.restoreFactoryDefaults();

        _clawEncoder = _claw.getEncoder();
        _clawEncoder.setPositionConversionFactor(ClawConstants.CLAW_POSITION_CONVERSION_FACTOR);

        _claw.setSmartCurrentLimit(ClawConstants.CLAW_STALL_LIMIT, ClawConstants.CLAW_FREE_LIMIT);
        _claw.setInverted(true);
        _claw.burnFlash();
        setBrakeMode();

    }

    public void resetEncoder() {
        _clawEncoder.setPosition(0);
    }

    public void periodic() {

      if (DriverStation.isTeleop()) {
        if ((Timer.getFPGATimestamp() - startedOuttakingTimestamp) > outtakeSeconds && startedOuttakingBool) {
            finishedOuttaking = true;
            stopClaw();
        }
      }
        
      setSpeed(desiredSpeed);
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

    
    public void setDesiredSpeed(double speed) {

        if (speed > 0) { intakeMode = true; }
        else if (speed < 0) { intakeMode = false; }

        if (intakeMode) {
            if (speed > this.desiredSpeed) {
                // To prevent damage to game elements, the claw will not intake at full speed
                this.desiredSpeed = speed * 0.8;
            }
        }
        else {
            this.desiredSpeed = speed;
        }
    }

    public void stopClaw() {
        this.desiredSpeed = 0;
    }

    public double getDesiredSpeed() {
        if (this.desiredSpeed > 0) { return (this.desiredSpeed / 0.7); }
        return this.desiredSpeed;
    }

    public void outTakeforXSeconds(double X) {
        setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED);
        this.startedOuttakingBool = true;
        this.outtakeSeconds = X;
        this.finishedOuttaking = false;
        this.startedOuttakingTimestamp = Timer.getFPGATimestamp();
    }

    public boolean getFinishedOuttaking() {
        return this.finishedOuttaking;
    }

    public void setStartedOuttakingBool(boolean startedOuttakingBool) {
      this.startedOuttakingBool = startedOuttakingBool;
    }

    public boolean getStartedOuttakingBool() {
      return this.startedOuttakingBool;
    }

    public void setFinishedOuttaking(boolean finishedOuttaking) {
        this.finishedOuttaking = finishedOuttaking;
    }

}
