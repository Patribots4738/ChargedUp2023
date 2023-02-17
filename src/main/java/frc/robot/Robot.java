// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import auto.AutoSegmentedWaypoints;
import auto.SwerveTrajectory;
import debug.Debug;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import hardware.Arm;
import hardware.Claw;
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
  Claw claw;

  Debug debug;

  @Override
  public void robotInit() {

    // Instantiate our Robot. This acts as a dictionary for all of our subsystems

    // Debug class for Shuffleboard
    debug = new Debug();

    // Drivetrain instantiation
    swerve = new Swerve();

    driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    arm = new Arm();
    claw = new Claw();

    autoSegmentedWaypoints = new AutoSegmentedWaypoints(swerve, arm, claw);
    autoSegmentedWaypoints.loadAutoPaths();

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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    SwerveTrajectory.resetTrajectoryStatus();

  }

  @Override
  public void autonomousPeriodic() {

    autoSegmentedWaypoints.periodic();
    arm.periodic();
    claw.periodic();

  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
