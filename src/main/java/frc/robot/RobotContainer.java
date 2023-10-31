package frc.robot;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlignment;
import frc.robot.commands.Drive;
import frc.robot.commands.auto.AutoSegmentedWaypoints;
import frc.robot.commands.auto.AutoPathStorage;
import frc.robot.commands.auto.AutoPathStorage.AutoPose;
import frc.robot.subsystems.ArduinoController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PhotonCameraUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;

public class RobotContainer {

    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private final Swerve swerve;
    private final Arm arm;
    private final Claw claw;
    private final PhotonCameraUtil photonVision;
    private final AutoAlignment autoAlignment;
    private final ArduinoController arduinoController;
    private final AutoPathStorage autoPathStorage;
    private final DriverUI driverUI = new DriverUI();
    
    public RobotContainer() {
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        swerve = new Swerve();
        arm = new Arm();
        claw = new Claw();
        photonVision = new PhotonCameraUtil();
        autoAlignment = new AutoAlignment(swerve, photonVision);
        arduinoController = new ArduinoController();
        autoPathStorage = new AutoPathStorage();

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.y().getAsBoolean(),
            () -> !driver.y().getAsBoolean()
        ));

        incinerateMotors();
        configureButtonBindings();

        // Wait wait wait wait for the DS to connect
        // then assign our alliance color
        while (DriverStation.getAlliance() == Alliance.Invalid) {
            DriverStation.refreshData();
        }

        FieldConstants.ALLIANCE = DriverStation.getAlliance();
    }

    private void configureButtonBindings() {
        
        driver.start().or(driver.back()).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    Rotation2d.fromDegrees(
                        FieldConstants.ALLIANCE == Alliance.Red 
                        ? 0 
                        : 180))
            ), swerve)
        );

        driver.a()
            .whileFalse(Commands.run(autoAlignment::setNearestValues))
            .whileTrue(
                Commands.sequence(
                    swerve.resetHDC(),
                    swerve.setAlignemntSpeed(),
                    // Run a method to get our desired speeds autonomously
                    // combine that with the driver axis X
                    // then run a swerve drive command with that
                    swerve.getAutoAlignmentCommand(
                        autoAlignment::getAutoAlignChassisSpeeds, 
                        () -> {
                            return ChassisSpeeds.fromFieldRelativeSpeeds(
                                driver.getLeftY(), 
                                0, 
                                0, 
                                swerve.getYaw()
                            );
                        }
                    )
                )
            )
            .onFalse(swerve.resetHDC());

        driver.leftBumper().whileTrue(Commands.run(swerve::getSetWheelsX));

        driver.rightStick().whileTrue(
            Commands.sequence(
                swerve.resetHDC(),
                swerve.getDriveCommand(
                    () -> {
                        return ChassisSpeeds.fromFieldRelativeSpeeds(
                            driver.getLeftY(),
                            driver.getLeftX(),
                            autoAlignment.getAngleSnapThetaSpeed(swerve.getClosest180Rotation2d()),
                            swerve.getYaw());
                    }, true, false)
            )
        );

        driver.leftStick().toggleOnTrue(swerve.toggleSpeed());

        driver.povLeft().onTrue(autoAlignment.coneOffsetLeft());
        driver.povRight().onTrue(autoAlignment.coneOffsetRight());

    }

    public Command getAutonomousCommand() {
        AutoPose chosenAutoPath;

        if (DriverUI.autoChooser.getSelected() == null) {
            chosenAutoPath = AutoPathStorage.myAutoContainer[0];
        } else {
            chosenAutoPath = DriverUI.autoChooser.getSelected();
        }
        return new AutoSegmentedWaypoints(swerve, arm, claw, autoAlignment, chosenAutoPath);
    }

    public Command getDisabledCommand() {
        return Commands.run(() -> FieldConstants.ALLIANCE = DriverStation.getAlliance()).ignoringDisable(true);
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkMax neo : NeoMotorConstants.motors) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }    
}
