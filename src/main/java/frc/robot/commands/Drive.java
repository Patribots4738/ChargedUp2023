package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Drive extends CommandBase {

    private final Swerve swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private final BooleanSupplier rateLimitSupplier;
    private final BooleanSupplier shouldMirror;
    
    public Drive(
            Swerve swerve, 
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationsSupplier, 
            BooleanSupplier rateLimitSupplier, 
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier shouldMirror) 
    {

        this.swerve = swerve;
        
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationsSupplier;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.rateLimitSupplier = rateLimitSupplier;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    public Drive (Swerve swerve, Supplier<ChassisSpeeds> speeds, BooleanSupplier rateLimitSupplier, BooleanSupplier fieldRelativeSupplier, BooleanSupplier shouldMirror) {
        
        this.swerve = swerve;
        
        this.xSupplier = () -> speeds.get().vxMetersPerSecond;
        this.ySupplier = () -> speeds.get().vyMetersPerSecond;
        this.rotationSupplier = () -> speeds.get().omegaRadiansPerSecond;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.rateLimitSupplier = rateLimitSupplier;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        if (shouldMirror.getAsBoolean()) {
            x *= -1;
            y *= -1;
        }
        swerve.drive(
            x,
            y,
            rotationSupplier.getAsDouble(), 
            fieldRelativeSupplier.getAsBoolean(), 
            rateLimitSupplier.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
