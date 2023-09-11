package hardware;

import auto.AutoAlignment;
import calc.Constants.ClawConstants;
import calc.Constants.PlacementConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Claw implements Loggable {

    private final CANSparkMax claw;
    private final RelativeEncoder clawEncoder;
    private double desiredSpeed = 0;
    private boolean intakeMode = false;

    // Timer values to have the claw auto outtake for X seconds
    private boolean startedOuttakingBool = false;
    private boolean finishedOuttaking = false;
    private double outtakeSeconds = 0;
    private double startedIntakingTimestamp = 0;
    private double startedOuttakingTimestamp = 0;
    @Log
    private boolean hasGameElement = false;
    @Log
    private double current = 0;
    @Log
    private boolean superIntake = false;

    public Claw() {

        claw = new CANSparkMax(ClawConstants.CLAW_CAN_ID, MotorType.kBrushless);
        claw.restoreFactoryDefaults();

        clawEncoder = claw.getEncoder();
        clawEncoder.setPositionConversionFactor(ClawConstants.CLAW_POSITION_CONVERSION_FACTOR);

        claw.setSmartCurrentLimit(ClawConstants.CLAW_CURRENT_LIMIT);
        claw.setInverted(true);
        claw.burnFlash();
        setBrakeMode();

    }

    public void resetEncoder() {
        clawEncoder.setPosition(0);
    }

    public void periodic() {

        // current = getOutputCurrent();

        // if (getOutputCurrent() > 30 && !hasGameElement && AutoAlignment.coneMode) {
        //     startedIntakingTimestamp = Timer.getFPGATimestamp();
        //     hasGameElement = true;
        //     claw.setSmartCurrentLimit(25);
        //     superIntake = true;
        // }

        // if ((Timer.getFPGATimestamp() - startedIntakingTimestamp > 0.25) && hasGameElement) {
        //     if (getOutputCurrent() < 2) { hasGameElement = false; }
        //     claw.setSmartCurrentLimit(15);
        //     superIntake = false;
        // }

        if (DriverStation.isTeleop()) {

            // This is for automatically outtaking the game piece
            if ((Timer.getFPGATimestamp() - startedOuttakingTimestamp) > outtakeSeconds && startedOuttakingBool) {
                finishedOuttaking = true;
            }
            // Slow down claw for cube mode
            if (!AutoAlignment.coneMode) {
              desiredSpeed = MathUtil.clamp(desiredSpeed, -0.3, 0.5);
            }
            // Speed up claw for cone mode
            else if (desiredSpeed > 0.45) {
                desiredSpeed = 1;
            }
        }
        setSpeed(desiredSpeed);
    }

    private void setSpeed(double speed) {
        claw.set(speed);
    }

    public void setBrakeMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    
    public void setDesiredSpeed(double speed) {

        if (speed > 0) { intakeMode = true; }
        else if (speed < 0) { intakeMode = false; }

        if (intakeMode) {
            if (speed > this.desiredSpeed) {
                // To prevent damage to game elements, the claw will not intake at full speed
                this.desiredSpeed = speed;
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

    public void outTakeforXSeconds(double X) {
        setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED_CONE);
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

    public double getOutputCurrent() {
        return claw.getOutputCurrent();
    }

}
