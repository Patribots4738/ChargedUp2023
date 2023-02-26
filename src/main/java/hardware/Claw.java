package hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import math.Constants.ClawConstants;
import math.Constants.PlacementConstants;

public class Claw {

    private final CANSparkMax _claw;
    private final RelativeEncoder _clawEncoder;
    private double desiredSpeed = 0;
    private boolean intakeMode = false;

    // Timer values to have the claw auto outtake for X seconds
    private boolean finishedOuttaking = false;
    private double outtakeSeconds = 0;
    private double startedOuttaking = 0;

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
        if ((Timer.getFPGATimestamp() - startedOuttaking) > outtakeSeconds) {
            finishedOuttaking = true;
            stopClaw();
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
                this.desiredSpeed = speed * 0.7;
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
        return this.desiredSpeed;
    }

    public void outTakeforXSeconds(double seconds) {
        setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED);
        this.outtakeSeconds = seconds;
        this.finishedOuttaking = false;
        this.startedOuttaking = Timer.getFPGATimestamp();
    }

    public boolean getFinishedOuttaking() {
        return this.finishedOuttaking;
    }

    public void setFinishedOuttaking(boolean finishedOuttaking) {
        this.finishedOuttaking = finishedOuttaking;
    }

}
