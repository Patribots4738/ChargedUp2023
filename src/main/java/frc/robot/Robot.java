package frc.robot;

import debug.*;
import edu.wpi.first.wpilibj.DriverStation;
import hardware.*;
import math.ArmCalculations;
import math.OICalc;
import math.Constants.*;
import auto.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import io.github.oblarg.oblog.Logger;
import subsystems.AutoAlignment;

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
      autoAlignment = new AutoAlignment(swerve);// Configure the logger for shuffleboard

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
  }

  @Override
  public void disabledPeriodic() {
    // SwerveTrajectory.resetHDC();
  }

  @Override
  public void autonomousInit() {

      autoSegmentedWaypoints.init();
      arm.setBrakeMode();
      SwerveTrajectory.resetTrajectoryStatus();

  }

  @Override
  public void autonomousPeriodic() {

    swerve.periodic();
    arm.periodic();
    claw.periodic();
    autoSegmentedWaypoints.periodic();
    autoAlignment.calibrateOdometry();

  }

  @Override
  public void teleopInit() {
    arm.setBrakeMode();
    SwerveTrajectory.resetTrajectoryStatus();
    autoAlignment.setConeOffset(0);
  }

  @Override
  public void teleopPeriodic() {

    arm.periodic();
    claw.periodic();

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

    // autoAlignment.calibrateOdometry();

    if (driver.getAButton()) {

      if (driver.getRightBumper()) {

        autoAlignment.moveToTag();

        if (autoAlignment.getMoveArmToHumanTag()) {
          arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
        }

    } else {

      SwerveTrajectory.resetTrajectoryStatus();

    }

    // Toggle the speed to be 10% of max speed when the driver's left stick is pressed
    if (driver.getRightStickButtonPressed()) {
      swerve.toggleSpeed();
    }

    } else if (driver.getLeftBumper()) {
      swerve.setWheelsX();
    } else {
      //              SpeedX,               SpeedY,              Rotation,    Field_Oriented
      if (driver.getYButton()) {
        swerve.drive(driverLeftAxis.getX(), -driverLeftAxis.getY(), -driverRightX * 0.25, false);
      }
      else {
        swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), -driverRightX * 0.25, true);
      }
    }

    // Toggle the operator override when the operator's left stick is pressed
    if (operator.getLeftStickButtonPressed()) {
      arm.toggleOperatorOverride();
    }
    if (arm.getOperatorOverride()) {
      arm.drive(new Translation2d(operatorLeftAxis.getX(), -operatorLeftAxis.getY()));
    }

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
        autoAlignment.setConeOffset(autoAlignment.getConeOffset() + ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        break;

      // Clicking right
      case 90:
        autoAlignment.setConeOffset(autoAlignment.getConeOffset() - ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        break;
    }

    switch (OICalc.getOperatorPOVPressed(operator.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        arm.setArmIndex((autoAlignment.getConeMode()) ? PlacementConstants.HIGH_CONE_PLACEMENT_INDEX : PlacementConstants.HIGH_CUBE_LAUNCH_INDEX);
        break;

      // Clicking down
      case 180:
        arm.setArmIndex(PlacementConstants.FLOOR_INTAKE_INDEX);
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
    } else if (operator.getLeftBumper()) {

      claw.stopClaw();

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
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    arm.periodic();
    if (driver.getAButtonPressed()) {
      autoAlignment.startChargePad();
    }
    if (driver.getAButton()) {
      autoAlignment.chargeAlign();
    }
    else {
      double driverLeftX = MathUtil.applyDeadband(-driver.getLeftX(), OIConstants.DRIVER_DEADBAND);
      double driverLeftY = MathUtil.applyDeadband(-driver.getLeftY(), OIConstants.DRIVER_DEADBAND);
      double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADBAND);
      double driverRightY = MathUtil.applyDeadband(driver.getRightY(), OIConstants.DRIVER_DEADBAND);
      Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);

      // If we are on blue alliance, flip the driverLeftAxis
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        driverLeftAxis = driverLeftAxis.unaryMinus();
      }
      //              SpeedX,               SpeedY,              Rotation,    Field_Oriented
      swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), -driverRightX * 0.25, !driver.getYButton());
    }
  }
}
