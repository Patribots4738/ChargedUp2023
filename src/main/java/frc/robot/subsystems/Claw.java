package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriverUI;
import frc.robot.util.Constants.ClawConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.PlacementConstants;

public class Claw extends SubsystemBase {

    private final CANSparkMax claw;
    private double desiredSpeed = 0;
    private boolean intakeMode = false;

    private double startedIntakingTimestamp = 0;
    private boolean hasGameElement = false;
    private boolean hasGameElementOneLoopBehind = false;
    // @Graph
    private double current = 0;

    public Claw() {

        claw = new CANSparkMax(ClawConstants.CLAW_CAN_ID, MotorType.kBrushless);
        claw.restoreFactoryDefaults();

        claw.setSmartCurrentLimit(ClawConstants.CLAW_CURRENT_LIMIT);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        claw.setInverted(false);

        NeoMotorConstants.motorList.add(claw);

        setBrakeMode();

    }

    @Override
    public void periodic() {

        /**
         * Output generally is -0.25 < appliedOutput < 0
         * When grabbygrab
         * Current is generally 15 < current < 30
         * Which is a big range unfortunately.
         * I am not sure where the current > 30 came from
         * I am having a hard time getting that read again on the graph anymore
         * Let's just go with > 15 for now
         * 
         * Lastly, desiredSpeed is ALWAYS > 0.45 when we are intaking
         * Which is a good indicator of finding if the claw is just getting up
         * to speed or if it is stalled
         *
         * The problem with having a boolean this sophisticated
         * Is that there are too many independent variables to check
         * And if a single one is off, the boolean will be false
         *
         * Idea: use a startedIntakingTimestamp to check
         * if the claw has been intaking for a while
         * i.e. is finished getting up to speed.
         */
        if (hasGameElement) {
            hasGameElement = desiredSpeed > 0;
        } else {
            hasGameElement = (-0.25 < claw.getAppliedOutput() && claw.getAppliedOutput() < 0) &&
                    (current > 15 && claw.getOutputCurrent() > 15) &&
                    (desiredSpeed > 0.45) &&
                    (Timer.getFPGATimestamp() - startedIntakingTimestamp > 0.25);
        }

        // Check if our current current and our current (one loop behind, hasn't updated
        // yet)
        // is in the range that happens when we have a game element,
        // but the claw is not yet in a stalled state.
        if (20 < current && current < 30 &&
                20 < claw.getOutputCurrent() && claw.getOutputCurrent() < 30 &&
                desiredSpeed > 0.45) {
            // Stop the claw temporarily
            // It will be restarted in the next loop
            claw.set(0);
            return;
        }

        DriverUI.spicyClaw = claw.getMotorTemperature() > 60;

        if (FieldConstants.GAME_MODE == FieldConstants.GameMode.TELEOP) {

            // Slow down claw for cube mode
            if (!PlacementConstants.CONE_MODE &&
                    (FieldConstants.GAME_MODE == FieldConstants.GameMode.TELEOP
                            || FieldConstants.GAME_MODE == FieldConstants.GameMode.TEST)) {
                desiredSpeed = MathUtil.clamp(desiredSpeed, -0.3, 0.5);
            }
            // Speed up claw for cone mode
            else if (desiredSpeed > 0.45) {
                desiredSpeed = 1;
            }
        }
        setSpeed(desiredSpeed);
        updateOutputCurrent();
    }

    private void setSpeed(double speed) {
        claw.set(-speed);
    }

    public void setBrakeMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoastMode() {
        claw.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setDesiredSpeed(double speed) {

        if (speed > 0) {
            intakeMode = true;
        } else if (speed <= 0) {
            intakeMode = false;
        }

        if (intakeMode) {
            if (speed > this.desiredSpeed) {
                // If our desiredSpeed is <= 0, that means that we just started intaking
                // So start a timer so that we can check if we have stalled
                // without taking into account the time it takes to speed up
                if (this.desiredSpeed <= 0) {
                    this.startedIntakingTimestamp = Timer.getFPGATimestamp();
                }

                // To prevent damage to game elements, the claw will not intake at full speed
                this.desiredSpeed = speed;
            }
        } else {
            this.desiredSpeed = speed;
        }
    }

    public void stopClaw() {
        this.desiredSpeed = 0;
    }

    public double getDesiredSpeed() {
        return this.desiredSpeed;
    }

    public Command outTakeForXSeconds(DoubleSupplier X) {
        return runOnce(() -> {
            setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED_CONE);
        }).andThen(Commands.waitSeconds(X.getAsDouble())
        .andThen(runOnce(() -> setDesiredSpeed(PlacementConstants.CLAW_STOPPED_SPEED))));
    }


    public void updateOutputCurrent() {
        current = claw.getOutputCurrent();
        hasGameElementOneLoopBehind = hasGameElement;
    }

    public boolean hasGameElement() {
        return hasGameElement;
    }

    public boolean justAcquiredGameElement() {
        return !hasGameElementOneLoopBehind && hasGameElement;
    }

    public Command setDesiredSpeedCommand(DoubleSupplier desiredSpeed) {
        return runOnce(() -> setDesiredSpeed(desiredSpeed.getAsDouble()));
    }

}
