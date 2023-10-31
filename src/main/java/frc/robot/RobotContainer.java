package frc.robot;

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
import frc.robot.util.Constants.LEDConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.PlacementConstants;

public class RobotContainer {

    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private final Swerve swerve;
    private final Arm arm;
    private final Claw claw;
    private final PhotonCameraUtil photonVision;
    private final AutoAlignment autoAlignment;
    private final ArduinoController arduinoController;
    @SuppressWarnings("unused")
    private final AutoPathStorage autoPathStorage;
    @SuppressWarnings("unused")
    private final DriverUI driverUI;
    
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
        driverUI = new DriverUI();

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.y().getAsBoolean(),
            () -> !driver.y().getAsBoolean()
        ));

        photonVision.setDefaultCommand(autoAlignment.calibrateOdometry());

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
            .whileTrue(
                Commands.sequence(
                    autoAlignment.setNearestValuesCommand(),
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

        operator.leftStick()
            .toggleOnTrue(arm.toggleOperatorOverride()
            .andThen(arm.getOperatorOverrideDriveCommand(operator::getLeftAxis)));

        operator.x().and(() -> PlacementConstants.CONE_MODE)
            .onTrue(autoAlignment.setConeModeFalse()
            .andThen(arduinoController.setLEDStateCommand(() -> LEDConstants.ARM_PURPLE))
            .andThen(arm.handleConeModeChange()));

        operator.y().and(() -> !PlacementConstants.CONE_MODE)
            .onTrue(autoAlignment.setConeModeTrue()
            .andThen(arduinoController.setLEDStateCommand(() -> LEDConstants.ARM_YELLOW))
            .andThen(arm.handleConeModeChange()));

        operator.povUp().onTrue(arm.getPOVHighCommand());
        operator.povDown().onTrue(arm.getPOVDownCommand());
        operator.povLeft().onTrue(arm.getPOVLeftCommand(() -> swerve.getPose().getX()));
        operator.povRight().onTrue(arm.getPOVRightCommand(() -> swerve.getPose().getX()));
        
        operator.a().whileTrue(arm.finishPlacmentCommand());
        
        operator.b()
            .onTrue(arm.getDriveCommand(() -> PlacementConstants.CONE_FLIP_INDEX)
            .until(arm::halfwayFinishedWithConeFlip)
            .andThen(claw.setDesiredSpeedCommand(() -> PlacementConstants.CLAW_INTAKE_SPEED_CONE)));

        operator.rightStick().and(() -> !arm.getAtPlacementPosition()).onTrue(arm.getDriveCommand(() -> PlacementConstants.STOWED_INDEX));

        operator.back().or(operator.start()).onTrue(arm.getUnflipCommand());

        operator.leftBumper().whileTrue(claw.setDesiredSpeedCommand(() -> PlacementConstants.CLAW_STOPPED_SPEED));

        operator.rightBumper().or(operator.rightStick().and(arm::getAtPlacementPosition))
            .onTrue(claw.outTakeforXSeconds(() -> PlacementConstants.CONE_MODE ? 0.1 : 0.3)
            .alongWith(arduinoController.setLEDStateCommand(() -> LEDConstants.BELLY_PAN_BLACK))
            .alongWith(arduinoController.setLEDStateCommand(
                () -> PlacementConstants.CONE_MODE ? 
                    LEDConstants.BELLY_PAN_YELLOW : 
                    LEDConstants.BELLY_PAN_PURPLE))
            .andThen(arm.getAutoStowCommand()));
        
        operator.leftTrigger().whileTrue(claw.setDesiredSpeedCommand(() -> operator.getLeftTriggerAxis()));
        operator.rightTrigger().whileTrue(claw.setDesiredSpeedCommand(() -> -operator.getRightTriggerAxis()));

        swerve.getTiltedTrigger().onTrue(arduinoController.setLEDStateCommand(() -> LEDConstants.BELLY_PAN_FLASH_RED));

        autoAlignment.getAlignedTrigger(() -> PlacementConstants.CONE_BASE_RADIUS)
            .onTrue(driver.setRumble(() -> 1)
            .andThen(operator.setRumble(() -> 1))
            .andThen(arduinoController.setLEDStateCommand(() -> 
                claw.hasGameElement() ? LEDConstants.BELLY_PAN_GREEN_BLINK :
                    PlacementConstants.CONE_MODE ? 
                        LEDConstants.BELLY_PAN_YELLOW : 
                        LEDConstants.BELLY_PAN_PURPLE)))
        .negate().and(autoAlignment.getAlignedTrigger(() -> 1)
            .onTrue(arduinoController.setLEDStateCommand(() -> 
                claw.hasGameElement() ? LEDConstants.BELLY_PAN_RED :
                    PlacementConstants.CONE_MODE ? 
                        LEDConstants.BELLY_PAN_YELLOW : 
                        LEDConstants.BELLY_PAN_PURPLE)))
        .negate().and(claw.hasGameElementTrigger())
            .onTrue(driver.setRumble(() -> 0.25)
            .andThen(operator.setRumble(() -> 0.25)))
            .onFalse(arduinoController.setLEDStateCommand(() ->
                PlacementConstants.CONE_MODE 
                    ? LEDConstants.BELLY_PAN_YELLOW 
                    : LEDConstants.BELLY_PAN_PURPLE))
        .and(claw::justAquiredGameElement)
            .onTrue(arduinoController.setLEDStateCommand(() ->
                PlacementConstants.CONE_MODE 
                    ? LEDConstants.BELLY_PAN_YELLOW_BLINK 
                    : LEDConstants.BELLY_PAN_PURPLE_BLINK))
            .onFalse(driver.setRumble(() -> 0)
            .andThen(operator.setRumble(() -> 0)));
    }

    public Command getAutonomousCommand() {
        boolean unfinished = true;
        if (unfinished) {
            return null;
        }
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
