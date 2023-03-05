package frc.robot;

import debug.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import hardware.*;
import calc.ArmCalculations;
import calc.OICalc;
import calc.Constants.*;
import auto.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import io.github.oblarg.oblog.Logger;
import auto.AutoAlignment;

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

  ArmCalculations armCalcuations;

  AutoAlignment autoAlignment;

  @Override
  public void robotInit() {
      // Instantiate our Robot. This acts as a dictionary for all of our subsystems

      // Debug class for ShuffleBoard
      debug = new Debug();

      // Drivetrain instantiation
      swerve = new Swerve();

      driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
      operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

      arm = new Arm();
      claw = new Claw();

      armCalcuations = new ArmCalculations();
      autoAlignment = new AutoAlignment(swerve, claw);// Configure the logger for shuffleboard

      autoSegmentedWaypoints = new AutoSegmentedWaypoints(swerve, arm, claw, autoAlignment);
      autoPathStorage = new AutoPathStorage();

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

      swerve.periodic();
      // arm.periodic();

      Logger.updateEntries();

  }

  @Override
  public void disabledInit() {
    // arm.setUpperArmCoastMode();
    claw.stopClaw();

    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);

    operator.setRumble(RumbleType.kLeftRumble, 0);
    operator.setRumble(RumbleType.kRightRumble, 0);

  }

  @Override
  public void disabledPeriodic() {
    // SwerveTrajectory.resetHDC();
  }

  @Override
  public void autonomousInit() {

    DriveConstants.MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
    autoSegmentedWaypoints.init();
    arm.setBrakeMode();
    SwerveTrajectory.resetTrajectoryStatus();

  }

  @Override
  public void autonomousPeriodic() {
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

    DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
    arm.setBrakeMode();
    SwerveTrajectory.resetTrajectoryStatus();
    autoAlignment.setConeOffset(0);

  }

  @Override
  public void teleopPeriodic() {

    // If we are in the last 100 ms of the match, set the wheels up
    // This is to prevent any charge pad sliding
    if (Timer.getMatchTime() < 0.1 && Timer.getMatchTime() != -1) {
      swerve.setWheelsUp();
      return;
    }

    arm.periodic();
    claw.periodic();
    autoAlignment.calibrateOdometry();

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

    if (driver.getAButton()) {

      if (driver.getAButtonPressed()) {

        DriveConstants.MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
        SwerveTrajectory.resetTrajectoryStatus();
        autoAlignment.setTagID(autoAlignment.getNearestTag());

      }

      autoAlignment.moveToTag();

      if (autoAlignment.getMoveArmToHumanTag()) {
        arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
      }

    } else if (driver.getAButtonReleased()) {

      DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;

    } else if (driver.getRightBumper()) {

      if (driver.getRightBumperPressed()) {

        autoAlignment.startChargePad();

      }

      autoAlignment.chargeAlign();

    } else if (driver.getLeftBumper()) {

      swerve.setWheelsX();

    } else {
      // If the driver holds the Y button, the robot will drive relative to itself
      // This is useful for driving in a straight line (backwards to intake!)
      if (driver.getYButton()) {
        swerve.drive(driverLeftAxis.getX(), driverLeftAxis.getY(), -driverRightX * 0.25, false);
      }
      else {
        // Flip the X and Y inputs to the swerve drive because going forward (up) is positive Y on a controller joystick
        swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), -driverRightX * 0.25, true);
      }
    }

    // Toggle the speed to be 10% of max speed when the driver's left stick is pressed
    if (driver.getRightStickButtonPressed()) {
      swerve.toggleSpeed();
    }

    // Toggle the operator override when the operator's left stick is pressed
    if (operator.getLeftStickButtonPressed()) {
      arm.toggleOperatorOverride();
    }
    
    if (arm.getOperatorOverride()) {

      // If the holonomic rotation is either positive, plus or minus 10 degrees on either side of 0 degrees (180/0)
      // Then set the operator X input to be negative
      if (swerve.getYaw().getDegrees() > 0 || 
         (swerve.getYaw().getDegrees() < 10 && swerve.getYaw().getDegrees() > -10) ||
          swerve.getYaw().getDegrees() < -170 && swerve.getYaw().getDegrees() > 170) 
      {
        arm.drive(new Translation2d(
          ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) 
              ? -operatorLeftAxis.getX() 
              : operatorLeftAxis.getX()), 
          -operatorLeftAxis.getY()));
      }
      else {
        arm.drive(new Translation2d(
          ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) 
              ? operatorLeftAxis.getX() 
              : -operatorLeftAxis.getX()), 
          -operatorLeftAxis.getY()));
      }

    }

    // The moment the robot takes in a cone/cube,
    // The operator can set the robot into the desired mode
    if (operator.getXButtonPressed()) {
      autoAlignment.setConeMode(false);
    }
    else if (operator.getYButtonPressed()) {
      autoAlignment.setConeMode(true);
    }

    if (operator.getStartButtonPressed()) {
      arm.setArmMirrored(true);
    }
    else if (operator.getBackButtonPressed()) {
      arm.setArmMirrored(false);
    }

    // POV = D-Pad...
    switch (OICalc.getDriverPOVPressed(driver.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        arm.setArmIndex(PlacementConstants.LONG_ARM_REACH_INDEX);
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
    switch (operator.getPOV()) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        arm.setArmIndex((autoAlignment.getConeMode()) ? PlacementConstants.HIGH_CONE_PLACEMENT_INDEX : PlacementConstants.HIGH_CUBE_LAUNCH_INDEX);
        break;

      // Clicking down
      case 180:
        arm.setArmIndex((autoAlignment.getConeMode()) ? PlacementConstants.CONE_INTAKE_INDEX : PlacementConstants.CUBE_INTAKE_INDEX);
        break;

      // Clicking left
      case 270:
        arm.setArmIndex((autoAlignment.getConeMode()) ? PlacementConstants.MID_CONE_PLACEMENT_INDEX : PlacementConstants.MID_CUBE_LAUNCH_INDEX);
        break;

      // Clicking right
      case 90:
        arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
        break;
    }

    if (operator.getRightStickButtonPressed()) {
    
      arm.setArmIndex(PlacementConstants.STOWED_INDEX);

    }

    if (operator.getStartButtonPressed()) {
      arm.setArmMirrored(true);
    }
    else if (operator.getBackButtonPressed()) {
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

    } else if (operator.getRightBumper() && !claw.getStartedOuttakingBool()) {
      // Check if the arm has completed the path to place an object
      if (arm.getAtPlacementPosition()) {

        claw.outTakeforXSeconds(0.5);

      }
    } else if (operator.getLeftTriggerAxis() > 0) {

      claw.setDesiredSpeed(operator.getLeftTriggerAxis());

    } else if (operator.getRightTriggerAxis() > 0) {

      claw.setDesiredSpeed(-operator.getRightTriggerAxis());

    } else {

      if (claw.getFinishedOuttaking() && arm.getAtPlacementPosition()) {

        if (arm.getArmIndex() == PlacementConstants.HIGH_CONE_PLACEMENT_INDEX) {
            arm.setArmIndex(PlacementConstants.HIGH_TO_STOWWED_INDEX);
        }
        else {
          arm.setArmIndex(PlacementConstants.STOWED_INDEX);
        }
          
        claw.setFinishedOuttaking(false);
        claw.setStartedOuttakingBool(false);
          
      }
    }

    // Controller rumble settings:
    if (arm.getAtPlacementPosition()) {
      operator.setRumble(RumbleType.kLeftRumble, 0.5);
      operator.setRumble(RumbleType.kRightRumble, 0.5);
    }
    else {
      operator.setRumble(RumbleType.kLeftRumble, 0);
      operator.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  @Override
  public void testInit() {
    autoAlignment.setTagID(3);
  }

  @Override
  public void testPeriodic() {
    if (driver.getRightBumper()) {

      if (driver.getRightBumperPressed()) {
        autoAlignment.startChargePad();
      }

      autoAlignment.chargeAlign();
    }

    // System.out.println(driver.getBackButton() + " " + driver.getStartButton());

    if (driver.getXButtonPressed()) {
      autoAlignment.setConeMode(false);
    }
    else if (driver.getYButtonPressed()) {
      autoAlignment.setConeMode(true);
    }

    switch (OICalc.getDriverPOVPressed(driver.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        arm.setArmIndex(PlacementConstants.LONG_ARM_REACH_INDEX);
        break;

      // Clicking down
      case 180:
        break;

      // Clicking left
      case 270:
        // If we are focusing on a substation, change the substation offset multiplier, not the cone offset multiplier.
        if (autoAlignment.getTagID() == 4 || autoAlignment.getTagID() == 5) {
          autoAlignment.setSubstationOffset((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? -1 : 1);
        }
        else {
          autoAlignment.setConeOffset(autoAlignment.getConeOffset() + ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        }
        break;

      // Clicking right
      case 90:
        // If we are focusing on a substation, change the substation offset multiplier, not the cone offset multiplier.
        if (autoAlignment.getTagID() == 4 || autoAlignment.getTagID() == 5) {
          autoAlignment.setSubstationOffset((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? -1 : 1);
        }
        else {
          autoAlignment.setConeOffset(autoAlignment.getConeOffset() - ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        }
        break;
    }
  }
}
