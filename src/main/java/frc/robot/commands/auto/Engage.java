package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriverUI;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.FieldConstants;

public class Engage extends CommandBase {
    
    private final Swerve swerve;
    private Supplier<Rotation2d> yaw;
    private Supplier<Rotation2d> pitch;
    private Supplier<Rotation2d> roll;

    private double tilt;

    private double startTimestamp = 0;
    private double elapsedTime = 0;

    public Engage(Swerve swerve, Supplier<Rotation2d> robotYaw, Supplier<Rotation2d> robotPitch, Supplier<Rotation2d> robotRoll) {
        this.swerve = swerve;
        
        this.yaw = robotYaw;
        this.pitch = robotPitch;
        this.roll = robotRoll;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startTimestamp = DriverUI.currentTimestamp;
    }

    @Override
    public void execute() {
        elapsedTime = DriverUI.currentTimestamp - startTimestamp;
        // If our heading is within -45 to 45 degrees or within -135 and -180 or within 135 to 180, use the pitch
        // Otherwise, use the roll
        if (-45 < yaw.get().getDegrees() && yaw.get().getDegrees() < 45) {
            tilt = -pitch.get().getRadians();
        }
        else if (-180 < yaw.get().getDegrees() && yaw.get().getDegrees() < -135 ||
            135 < yaw.get().getDegrees() && yaw.get().getDegrees() < 180) 
        {
            tilt = pitch.get().getRadians();
        }
        else if (-135 < yaw.get().getDegrees() && yaw.get().getDegrees() < -45) {
            tilt = roll.get().getRadians();
        }
        else if (45 < yaw.get().getDegrees() && yaw.get().getDegrees() < 135) 
        {
            tilt = -roll.get().getRadians();
        }
        
        if (tilt > Math.toRadians(7)) {
            swerve.drive(
                MathUtil.clamp(
                    (
                        (FieldConstants.CHARGE_PAD_CORRECTION_P * tilt)
                        /
                        (elapsedTime /
                            (FieldConstants.GAME_MODE == FieldConstants.GameMode.AUTONOMOUS ? 10 : 20))), 0.1, 0.35),
                0, 
                0,
                true, false);
        }
        else if (tilt < -Math.toRadians(7)) {
            swerve.drive(
                MathUtil.clamp(
                    (
                        (FieldConstants.CHARGE_PAD_CORRECTION_P * tilt)
                        /
                        (elapsedTime / 
                            (FieldConstants.GAME_MODE == FieldConstants.GameMode.AUTONOMOUS ? 10 : 20))), -0.35, -0.1),
                0, 
                0, 
                true, false);
        }
        else {
            swerve.setWheelsUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setWheelsUp();
    }

    @Override
    public boolean isFinished() {
        // return false because we want to run this forever and ever???
        return Math.abs(tilt) < Math.toRadians(7) && elapsedTime > 5;
    }
}
