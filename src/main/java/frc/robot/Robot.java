// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import auto.AutoSegmentedWaypoints;
import auto.SwerveTrajectory;
import debug.Debug;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import hardware.Arm;
import hardware.Swerve;
import io.github.oblarg.oblog.Logger;
import math.Constants.OIConstants;
import math.OICalc;
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
  AutoAlignment autoAlignment;

  Arm arm;

  // Debug debug;

  @Override
  public void robotInit() {

    // Instantiate our Robot. This acts as a dictionary for all of our subsystems

    // Debug class for Shuffleboard
    // debug = new Debug();

    // Drivetrain instantiation
    swerve = new Swerve();

    driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    arm = new Arm();

    autoSegmentedWaypoints = new AutoSegmentedWaypoints(swerve, arm);
    autoSegmentedWaypoints.loadAutoPaths();

    autoAlignment = new AutoAlignment(swerve);

    // Configure the logger for shuffleboard
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
    arm.setUpperArmCoastMode();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    SwerveTrajectory.resetTrajectoryStatus();

  }

  @Override
  public void autonomousPeriodic() {

    autoSegmentedWaypoints.periodic();
    // arm.periodic();

  }

  @Override
  public void teleopInit() {
    arm.setBrakeMode();
  }

  @Override
  public void teleopPeriodic() {
    // arm.periodic();

    // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
    // This is to ensure that the up direction on the joystick is positive inputs
    double driverLeftX    = MathUtil.applyDeadband(driver.getLeftX()   , OIConstants.DRIVER_DEADBAND);
    double driverLeftY    = MathUtil.applyDeadband(-driver.getLeftY()  , OIConstants.DRIVER_DEADBAND);
    double driverRightX   = MathUtil.applyDeadband(driver.getRightX()  , OIConstants.DRIVER_DEADBAND);
    double driverRightY   = MathUtil.applyDeadband(driver.getRightY()  , OIConstants.DRIVER_DEADBAND);

    double operatorLeftX  = MathUtil.applyDeadband(operator.getLeftX() , OIConstants.DRIVER_DEADBAND);
    double operatorLeftY  = MathUtil.applyDeadband(operator.getLeftY() , OIConstants.DRIVER_DEADBAND);
    double operatorRightX = MathUtil.applyDeadband(operator.getRightX(), OIConstants.DRIVER_DEADBAND);
    double operatorRightY = MathUtil.applyDeadband(operator.getRightY(), OIConstants.DRIVER_DEADBAND);

    Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);

    // If we are on blue alliance, flip the driverLeftAxis
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      driverLeftAxis = driverLeftAxis.unaryMinus();
    }
    Translation2d operatorLeftAxis = OICalc.toCircle(operatorLeftX, operatorLeftY);

    if (driver.getRightBumper()) {
      swerve.setX();
    } else {
      //              SpeedX,               SpeedY,              Rotation,    Field_Oriented
      swerve.drive(driverLeftAxis.getX(), driverLeftAxis.getY(), driverRightX, true);
    }

    // Toggle the speed to be 10% of max speed when the driver's left stick is pressed
    if (driver.getLeftStickButtonPressed()) {
      swerve.toggleSpeed();
    }

    // Toggle the operator override when the operator's left stick is pressed
    // if (operator.getLeftStickButtonPressed()) {
    //   arm.toggleOperatorOverride();
    // // }
    // if (arm.getOperatorOverride()) {
    //   arm.drive(new Translation2d(operatorLeftAxis.getX(), operatorLeftAxis.getY()));
    // }
    // else if (operator.getRightBumperPressed()) {
    //   arm.setArmIndex(arm.getArmIndex() + 1);
    // }
    // else if (operator.getLeftBumperPressed()) {
    //   arm.setArmIndex(arm.getArmIndex() - 1);
    // }

  }

  @Override
  public void testInit() {
    swerve.resetEncoders();
    swerve.setBrakeMode();

    arm.setBrakeMode();

    SwerveTrajectory.resetTrajectoryStatus();

    swerve.resetOdometry(new Pose2d(11.07, 4.69, Rotation2d.fromDegrees(0)));

  }

  @Override
  public void testPeriodic() {

    
    // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
    // This is to ensure that the up direction on the joystick is positive inputs
    double driverLeftX  = MathUtil.applyDeadband(driver.getLeftX() , OIConstants.DRIVER_DEADBAND);
    double driverLeftY  = MathUtil.applyDeadband(-driver.getLeftY(), OIConstants.DRIVER_DEADBAND);
    double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADBAND);
    double driverRightY = MathUtil.applyDeadband(driver.getRightY(), OIConstants.DRIVER_DEADBAND);

    Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);
    
    // If we are on blue alliance, flip the driverLeftAxis
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      driverLeftAxis = driverLeftAxis.unaryMinus();
    }
    // Use the A button to activate the alignment process
    if (driver.getAButton()) {

      if (driver.getRightBumperPressed()) {
        System.out.println("Swerve Before Align: " + swerve.getPose() + "\n\n");

        autoAlignment.calibrateOdometry();

        System.out.println("Swerve After Align: " + swerve.getPose() + "\n\n");

      }

      autoAlignment.moveToTag();

      if (driver.getRightBumper()) {
        switch (driver.getPOV()) {
          // Not clicked
          case -1:
            break;

          // Clicking up
          case 0:
            // arm.setArmIndex(arm.getArmIndex() + 1);
            break;

          // Clicking down
          case 180:
            // arm.setArmIndex(arm.getArmIndex() - 1);
            break;

          // Clicking left
          case 270:
            autoAlignment.setConeOffset(autoAlignment.getConeOffset() - 1);
            break;

          // Clicking right
          case 90:
            autoAlignment.setConeOffset(autoAlignment.getConeOffset() + 1);
            break;
        }
      }

    } else {

      swerve.drive(driverLeftAxis.getX(), driverLeftAxis.getY(), driverRightX * .25, true);
      System.out.println(swerve.getPose());
    }
  }
}
