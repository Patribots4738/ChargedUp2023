// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import hardware.*;
import math.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  Swerve swerve;
  XboxController driver;
  XboxController operator;

  // Arm arm;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our Robot. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    swerve = new Swerve();

    driver = new XboxController(Constants.kDriverControllerPort);
    operator = new XboxController(Constants.kOperatorControllerPort);
  
    // arm = new Arm();
   
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    // Update the odometry for the swerve drive
    swerve.periodic();

    // Drive the robot
    //               SpeedX             SpeedY            Rotation         Field Relative
    swerve.drive(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), isAutonomous());
    
    // arm.drive(operator.getRightX(), operator.getRightY());

  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}