package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlignment;
import frc.robot.commands.Drive;
import frc.robot.commands.auto.AutoPathStorage;
import frc.robot.commands.auto.AutoPathStorage.AutoMap;
import frc.robot.subsystems.ArduinoController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PhotonCameraUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LEDConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.PlacementConstants;
import frc.robot.util.Constants.FieldConstants.GameMode;
import io.github.oblarg.oblog.Logger;

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
    private final DriverUI driverUI = new DriverUI();
    
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);

        DriverStation.silenceJoystickConnectionWarning(true);

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        swerve = new Swerve();
        arm = new Arm();
        claw = new Claw();
        photonVision = new PhotonCameraUtil();
        autoPathStorage = new AutoPathStorage();
        autoAlignment = new AutoAlignment(swerve, photonVision);
        arduinoController = new ArduinoController();

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.y().getAsBoolean(),
            () -> !driver.y().getAsBoolean(),
            () -> (driver.y().getAsBoolean() && FieldConstants.ALLIANCE == Alliance.Blue)
        ));

        photonVision.setDefaultCommand(autoAlignment.calibrateOdometry());

        incinerateMotors();
        configureButtonBindings();
        generateEventMap();

        // Wait wait wait wait for the DS to connect
        // then assign our alliance color
        while (DriverStation.getAlliance() == Alliance.Invalid) {
            DriverStation.refreshData();
        }

        FieldConstants.ALLIANCE = DriverStation.getAlliance();

        Commands.runOnce(() -> DriverUI.currentTimestamp = Timer.getFPGATimestamp())
            .andThen(Logger::updateEntries)
            .ignoringDisable(true)
            .repeatedly()
            .schedule();
    }

    private void configureButtonBindings() {

        new Trigger(() -> FieldConstants.GAME_MODE == FieldConstants.GameMode.TELEOP || FieldConstants.GAME_MODE == FieldConstants.GameMode.TEST)
            .onTrue(
                setDriveSpeed(DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND)
            );

        new Trigger(() -> FieldConstants.GAME_MODE == FieldConstants.GameMode.AUTONOMOUS)
            .onTrue(
                setDriveSpeed(AutoConstants.MAX_SPEED_METERS_PER_SECOND)
            );

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
                    setDriveSpeed(FieldConstants.ALIGNMENT_SPEED),
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
            ).onFalse(
                setDriveSpeed(DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND)
            );

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
        
        operator.a().whileTrue(
            arm.finishPlacmentCommand()
            .unless(
                () -> !(arm.getAtPrepIndex() && arm.getAtDesiredPositions()))
            .repeatedly());
        
        operator.b()
            .onTrue(arm.setArmIndexCommand(() -> PlacementConstants.CONE_FLIP_INDEX)
            .until(arm::halfwayFinishedWithConeFlip)
            .andThen(claw.setDesiredSpeedCommand(() -> PlacementConstants.CLAW_INTAKE_SPEED_CONE)));

        operator.rightStick().and(() -> !arm.getAtPlacementPosition()).onTrue(arm.setArmIndexCommand(() -> PlacementConstants.STOWED_INDEX));

        operator.back().or(operator.start()).onTrue(arm.getUnflipCommand());

        operator.leftBumper().whileTrue(claw.setDesiredSpeedCommand(() -> PlacementConstants.CLAW_STOPPED_SPEED));

        operator.rightBumper().or(operator.rightStick()).and(arm::getAtPlacementPosition)
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
        .negate().and(claw::hasGameElement)
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

    private CommandBase setDriveSpeed(double desiredSpeedMetersPerSecond) {
        return Commands.runOnce(() -> { 
            DriveConstants.MAX_SPEED_METERS_PER_SECOND = desiredSpeedMetersPerSecond; 
        });
    }

    public void generateEventMap() {

        AutoConstants.EVENT_MAP.put("Stow", arm.getAutoStowCommand());

        AutoConstants.EVENT_MAP.put("CubeIntake", 
            arm.setArmIndexCommand(() -> PlacementConstants.CUBE_INTAKE_INDEX)
            .alongWith(claw.setDesiredSpeedCommand(() -> PlacementConstants.CLAW_INTAKE_SPEED_CUBE))
        );

        AutoConstants.EVENT_MAP.put("IntakeCone", 
            claw.setDesiredSpeedCommand(() -> PlacementConstants.CLAW_INTAKE_SPEED_CONE)
        );

        AutoConstants.EVENT_MAP.put("PlaceHighCube", arm.placeHighCubeCommand());
        AutoConstants.EVENT_MAP.put("PlaceHighCone", arm.getPOVHighCommand());
        AutoConstants.EVENT_MAP.put("PlaceMidCube", arm.getPOVLeftCommand(() -> swerve.getPose().getX()));

        AutoConstants.EVENT_MAP.put("PlacePiece",
            arm.finishPlacmentCommand()
            .andThen(claw.outTakeforXSeconds(() -> 0.15))
            .andThen(arm.getAutoStowCommand())
        );

    }

    public Command getAutonomousCommand() {
        AutoMap chosenAutoPath;

        if (DriverUI.autoChooser.getSelected() == null) {
            chosenAutoPath = AutoPathStorage.myAutoContainer[0];
        } else {
            chosenAutoPath = DriverUI.autoChooser.getSelected();
            if (chosenAutoPath.getName().contains("LOW")) {
                AutoConstants.EVENT_MAP.put("PlaceMidCube", arm.getAutoStowCommand());
            } else {
                AutoConstants.EVENT_MAP.put("PlaceMidCube", arm.getPOVLeftCommand(() -> swerve.getPose().getX()));
            }
        }
        return swerve.fullAuto(() -> chosenAutoPath.getTrajectories());
    }

    public Command getDisabledCommand() {
        return Commands.run(() -> FieldConstants.ALLIANCE = DriverStation.getAlliance()).ignoringDisable(true);
    }

    public void onEnabled(GameMode gameMode) {
        Commands.runOnce(() -> DriverUI.modeStartTimestamp = DriverUI.currentTimestamp)
            .andThen(Commands.runOnce(() -> FieldConstants.GAME_MODE = gameMode))
            .schedule();
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkMax neo : NeoMotorConstants.motorList) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }    
}
