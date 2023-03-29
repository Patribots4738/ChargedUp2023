package frc.robot;

import auto.AutoAlignment;
import auto.AutoPathStorage;
import auto.AutoSegmentedWaypoints;
import auto.SwerveTrajectory;
import calc.ArmCalculations;
import calc.Constants.AlignmentConstants;
import calc.Constants.AutoConstants;
import calc.Constants.DriveConstants;
import calc.Constants.LEDConstants;
import calc.Constants.OIConstants;
import calc.Constants.PlacementConstants;
import calc.OICalc;
import debug.Debug;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import hardware.ArduinoController;
import hardware.Arm;
import hardware.Claw;
import hardware.Swerve;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // The robot's subsystems and commands are defined here...

  Swerve swerve;
  
  XboxController driver;
  XboxController operator;
  
  AutoSegmentedWaypoints autoSegmentedWaypoints;
  AutoPathStorage autoPathStorage;
  
  Arm arm;
  Claw claw;

  Debug debug;

  ArduinoController arduinoController;
  ArmCalculations armCalculations;

  AutoAlignment autoAlignment;

  @Override
  public void robotInit() {
      // Instantiate our Robot. This acts as a dictionary for all of our subsystems

      // Debug class for ShuffleBoard
      // debug = new Debug();

      // Drivetrain instantiation
      swerve = new Swerve();

      driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
      operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

      arm = new Arm();
      claw = new Claw();

      armCalculations = new ArmCalculations();
      autoAlignment = new AutoAlignment(swerve, claw);// Configure the logger for shuffleboard

      autoSegmentedWaypoints = new AutoSegmentedWaypoints(swerve, arm, claw, autoAlignment);
      autoPathStorage = new AutoPathStorage();

      arduinoController = new ArduinoController();

      Logger.configureLoggingAndConfig(this, false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Used for items like diagnostics
   * ran during disabled, autonomous, teleoperated and test. :D
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
        
    Logger.updateEntries();
    arduinoController.periodic();

  }

  @Override
  public void disabledInit() {
    // arm.setUpperArmCoastMode();
    claw.stopClaw();

    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);

    operator.setRumble(RumbleType.kLeftRumble, 0);
    operator.setRumble(RumbleType.kRightRumble, 0);
    
    arduinoController.setLEDState(LEDConstants.ARM_RAINBOW);
    arduinoController.setLEDState(DriverStation.getAlliance() == Alliance.Blue ? LEDConstants.BELLY_PAN_BLUE : LEDConstants.BELLY_PAN_RED_ALLIANCE);

    swerve.setSpeedMultiplier(1);

    System.out.println(swerve.getPose().getTranslation());

  }

  @Override
  public void disabledPeriodic() {
    if (Math.abs(swerve.getPitch().getDegrees()) > 35) {
      arduinoController.setLEDState(LEDConstants.BELLY_PAN_FLASH_RED);
    }
    else {
      arduinoController.setLEDState(DriverStation.getAlliance() == Alliance.Blue ? LEDConstants.BELLY_PAN_BLUE : LEDConstants.BELLY_PAN_RED_ALLIANCE);
    }
  }

  @Override
  public void autonomousInit() {

    DriveConstants.MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
    autoAlignment.setConeMode(true);
    arm.setBrakeMode();
    autoSegmentedWaypoints.init();
    SwerveTrajectory.resetTrajectoryStatus();

  }

  @Override
  public void autonomousPeriodic() {
    swerve.periodic();
    // System.out.printf("Time Left %.1f\n", Timer.getMatchTime());
    // If we are in the last 100 ms of the match, set the wheels up
    // This is to prevent any charge pad sliding
    if (Timer.getMatchTime() < 0.1 && Timer.getMatchTime() != -1) {
      swerve.setWheelsUp();
      return;
    }

    swerve.periodic();
    arm.periodic();
    claw.periodic();
    autoSegmentedWaypoints.periodic();
    autoAlignment.calibrateOdometry();
  }

  @Override
  public void teleopInit() {
    autoAlignment.setConeMode(true);
    arduinoController.setLEDState(LEDConstants.ARM_YELLOW);
    DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
    arm.setBrakeMode();
    SwerveTrajectory.resetTrajectoryStatus();
    autoAlignment.setConeOffset(0);
  }

  @Override
  public void teleopPeriodic() {

    swerve.periodic();
    arm.periodic();
    claw.periodic();
    autoAlignment.calibrateOdometry();

    // If we are in the last 100 ms of the match, set the wheels up
    // This is to prevent any charge pad sliding
    if (Timer.getMatchTime() < 0.1 && Timer.getMatchTime() != -1) {
      swerve.setWheelsUp();
      claw.setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED_CONE);
      return;
    }

    // Find the max angular velocity of the drivetrain using the arm location
    DriveConstants.DYNAMIC_MAX_ANGULAR_SPEED = armCalculations.lerp(
      DriveConstants.MIN_ANGULAR_SPEED, 
      DriveConstants.MAX_ANGULAR_SPEED,
      MathUtil.clamp(1-(arm.getXPosition()/40.0), 0, 1)
    );

    // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
    // This is to ensure that the up direction on the joystick is positive inputs
    double driverLeftX = MathUtil.applyDeadband(-driver.getLeftX(), OIConstants.DRIVER_DEADBAND);
    double driverLeftY = MathUtil.applyDeadband(-driver.getLeftY(), OIConstants.DRIVER_DEADBAND);
    double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADBAND);
    double driverRightY = MathUtil.applyDeadband(driver.getRightY(), OIConstants.DRIVER_DEADBAND);

    double operatorLeftX = MathUtil.applyDeadband(operator.getLeftX(), OIConstants.OPERATOR_DEADBAND);
    double operatorLeftY = MathUtil.applyDeadband(operator.getLeftY(), OIConstants.OPERATOR_DEADBAND);
    double operatorRightX = MathUtil.applyDeadband(operator.getRightX(), OIConstants.OPERATOR_DEADBAND);
    double operatorRightY = MathUtil.applyDeadband(operator.getRightY(), OIConstants.OPERATOR_DEADBAND);

    Translation2d operatorLeftAxis = OICalc.toCircle(operatorLeftX, operatorLeftY);

    Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);

    // If we are on blue alliance, flip the driverLeftAxis
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      driverLeftAxis = driverLeftAxis.unaryMinus();
    }

    if (SwerveTrajectory.trajectoryStatus.equals("setup") && !driver.getAButton()) {
      autoAlignment.setNearestValues();
    }

    // When not aligning, reset the max speed to the teleop speed
    if (driver.getAButtonReleased()) {
      DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
      SwerveTrajectory.resetTrajectoryStatus();
      SwerveTrajectory.HDC.getThetaController().reset(swerve.getYaw().getRadians());
      //autoAlignment.setConeOffset(AutoAlignment.coneMode ? 1 : 0);
    }
    else if (driver.getAButton()) {

      if (driver.getAButtonPressed()) {

        DriveConstants.MAX_SPEED_METERS_PER_SECOND = AlignmentConstants.MAX_SPEED_METERS_PER_SECOND;
        SwerveTrajectory.resetTrajectoryStatus();
        SwerveTrajectory.HDC.getThetaController().reset(swerve.getYaw().getRadians());
        
      }
      
      autoAlignment.alignToTag(driverLeftAxis.getY());

    } else if (driver.getRightBumper()) {

      if (driver.getRightBumperPressed()) {

        autoAlignment.startChargePad();

      }

      autoAlignment.chargeAlign();

    // Use the left bumper as a sort of e-stop for the swerve
    } else if (driver.getLeftBumper()) {

      swerve.setWheelsX();

    } else {
      // If the driver holds the left stick button, the robot will snap to the nearest 180 degree angle
      if (driver.getRightStickButton()) {
        if (driver.getRightStickButtonPressed()) {
          SwerveTrajectory.HDC.getThetaController().reset(swerve.getYaw().getRadians());
        }
        autoAlignment.snapToAngle(driverLeftAxis, Rotation2d.fromDegrees(Math.abs(swerve.getYaw().getDegrees()) > 90 ? 180 : 0));
      }
      // If the driver holds the Y button, the robot will drive relative to itself
      // This is useful for driving in a straight line (backwards to intake!)
      else if (driver.getYButton()) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          swerve.drive(-driverLeftAxis.getX(), -driverLeftAxis.getY(), -driverRightX * 0.25, false, false);
        }
        else {
          swerve.drive(driverLeftAxis.getX(), driverLeftAxis.getY(), -driverRightX * 0.25, false, false);
        }
      }
      else {
        // Flip the X and Y inputs to the swerve drive because going forward (up) is positive Y on a controller joystick
        swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), -driverRightX * 0.25, true, true);
      }
    }

    // Toggle the speed to be 10% of max speed when the driver's left stick is pressed
    if (driver.getLeftStickButtonPressed()) {
      swerve.toggleSpeed();
    }

    // Toggle the operator override when the operator's left stick is pressed
    if (operator.getLeftStickButtonPressed()) {
      arm.toggleOperatorOverride();
    }
    
    if (arm.getOperatorOverride()) {
      
      arm.drive(new Translation2d(-operatorLeftAxis.getX(), -operatorLeftAxis.getY()));
      
    }

    // The moment the robot takes in a cone/cube,
    // The operator can set the robot into the desired mode
    if (operator.getXButton()) {
      autoAlignment.setConeMode(false);
      arduinoController.setLEDState(LEDConstants.ARM_PURPLE);
    }
    else if (operator.getYButton()) {
      autoAlignment.setConeMode(true);
      arduinoController.setLEDState(LEDConstants.ARM_YELLOW);
    }

    // POV = D-Pad...
    switch (OICalc.getDriverPOVPressed(driver.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        break;

      // Clicking down
      case 180:
        break;

      // Clicking left
      case 270:
        // If we are focusing on a substation, change the substation offset multiplier, not the cone offset multiplier.
        if (autoAlignment.getTagID() == 4 || autoAlignment.getTagID() == 5) {
          autoAlignment.setSubstationOffset((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1);
        }
        else {
          autoAlignment.setConeOffset(autoAlignment.getConeOffset() + ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        }
        break;

      // Clicking right
      case 90:
        // If we are focusing on a substation, change the substation offset multiplier, not the cone offset multiplier.
        if (autoAlignment.getTagID() == 4 || autoAlignment.getTagID() == 5) {
          autoAlignment.setSubstationOffset((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1);
        }
        else {
          autoAlignment.setConeOffset(autoAlignment.getConeOffset() - ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        }
        break;
    }

    // POV = D-Pad...
    switch (OICalc.getOperatorPOVPressed(operator.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.HIGH_CONE_PREP_INDEX : PlacementConstants.HIGH_CUBE_LAUNCH_INDEX);
        break;

      // Clicking down
      case 180:
        arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_INTAKE_INDEX : PlacementConstants.CUBE_INTAKE_INDEX);
        break;

      // Clicking left
      case 270:
        arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.MID_CONE_PREP_INDEX : PlacementConstants.MID_CUBE_LAUNCH_INDEX);
        break;

      // Clicking right
      case 90:
        arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
        break;
    }
    if (operator.getBButtonPressed()){
      arm.setArmIndex(PlacementConstants.CONE_FLIP_INDEX);
    }
    if (operator.getRightStickButtonPressed()) {
      arm.setArmIndex(PlacementConstants.STOWED_INDEX);
    }
    if (operator.getAButtonPressed()) {
      arm.finishPlacement();
    }

    // These buttons are right below the Xbox logo, aimed to be a bit out of reach
    // (after all we don't want to accidentally press them)
    if (operator.getBackButtonPressed()) {
      arm.setArmMirrored(true);
    }
    else if (operator.getStartButtonPressed()) {
      arm.setArmMirrored(false);
    }

    // Claw speed controls
      // If the left bumper is pressed, e-stop the claw
      // If the right bumper is held, outtake an object when the arm is at placement position
      // If the left trigger is held, intake an object
        // Keep the fastest intake speed until the claw is e-stopped/reversed
        // This is to allow the trigger to be fully pressed intake an object,
      // and then let go to keep the claw at the same speed
      // If the right trigger is held, manually outtake an object (try to use the right bumper instead)
    if (operator.getLeftBumper()) {

      claw.stopClaw();

    } else if ((operator.getRightBumper()) && !claw.getStartedOuttakingBool()) {
      // Check if the arm has completed the path to place an object
      if (arm.getAtPlacementPosition()) {
        claw.outTakeforXSeconds(AutoAlignment.coneMode ? 0.1 : 0.3);
      }

    } else if (operator.getLeftTriggerAxis() > 0) {

      claw.setDesiredSpeed(operator.getLeftTriggerAxis());

    } else if (operator.getRightTriggerAxis() > 0) {

      claw.setDesiredSpeed(-operator.getRightTriggerAxis());

    } else if (claw.getFinishedOuttaking() && arm.getAtPlacementPosition()) {

      if (arm.getArmIndex() == PlacementConstants.HIGH_CONE_PLACEMENT_INDEX ||
          arm.getArmIndex() == PlacementConstants.HIGH_CONE_PREP_TO_PLACE_INDEX) {
          arm.setArmIndex(PlacementConstants.HIGH_TO_STOWWED_INDEX);
      }
      else {
          arm.setArmIndex(PlacementConstants.STOWED_INDEX);
      }

      claw.setStartedOuttakingBool(false);
      claw.setFinishedOuttaking(false);
      claw.stopClaw();

    }

    // Controller rumble settings:
    // If the robot is close to the desired grid index, rumble both controllers
    // This is to help the driver know when to stop moving the robot if manually aligning
    // Or as a confirmation for auto alignment
    if (Math.abs(swerve.getPitch().getDegrees()) > 35) {
      arduinoController.setLEDState(LEDConstants.BELLY_PAN_FLASH_RED);
    }
    else if (autoAlignment.getCurrentNorm() < (PlacementConstants.CONE_BASE_DIAMETER/2) && (autoAlignment.getCurrentNorm() != -1)) {
      driver.setRumble(RumbleType.kLeftRumble, 0.75);
      driver.setRumble(RumbleType.kRightRumble, 0.75);
      operator.setRumble(RumbleType.kRightRumble, 0.75);
      operator.setRumble(RumbleType.kLeftRumble, 0.75);
      arduinoController.setLEDState(LEDConstants.BELLY_PAN_GREEN);
    }
    else {
      if (autoAlignment.getCurrentNorm() < 1 && (autoAlignment.getCurrentNorm() != -1)) {
        arduinoController.setLEDState(LEDConstants.BELLY_PAN_RED);
      }
      else {
        arduinoController.setLEDState(AutoAlignment.coneMode ? LEDConstants.BELLY_PAN_YELLOW : LEDConstants.BELLY_PAN_PURPLE);
      }
      driver.setRumble(RumbleType.kLeftRumble, 0);
      driver.setRumble(RumbleType.kRightRumble, 0);
      operator.setRumble(RumbleType.kRightRumble, 0);
      operator.setRumble(RumbleType.kLeftRumble, 0);
    }

    //Rumble the claw if it is stalling, judged by whether or not the claw is drawing more amps than a preset limit.
    if (claw.getOutputCurrent() > 25) {
      driver.setRumble(RumbleType.kBothRumble, 0.1);
      operator.setRumble(RumbleType.kBothRumble, 0.1);
    }

  }  

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    if (operator.getXButton()) {
      autoAlignment.setConeMode(false);
      arduinoController.setLEDState(LEDConstants.ARM_PURPLE);
    }
    else if (operator.getYButton()) {
      autoAlignment.setConeMode(true);
      arduinoController.setLEDState(LEDConstants.ARM_YELLOW);
    }

    
  }
}
