// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import hardware.*;
import math.Constants.*;
import auto.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // The robot's subsystems and commands are defined here...
  // ExampleSubsystem exampleSubsystem; 
  Swerve swerve;

  XboxController driver;
  XboxController operator;
  
  AutoWaypoints autoWaypoints;

  // Arm arm;
  
  HolonomicDriveController autoController;

  Trajectory trajectory;
  TestPath autoPath;

  Debug debug;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Instantiate our Robot. This acts as a dictionary for all of our subsystems
    
    // Debug class so we can use shuffleboard
    debug = new Debug();

    // Drivetrain instantiation
    swerve = new Swerve();
    // Zero the IMU for field-oriented driving
    swerve.zeroHeading();

    // Setup controllers
    driver = new XboxController(OIConstants.kDriverControllerPort);
    operator = new XboxController(OIConstants.kOperatorControllerPort);

    // Arm Instantiation
    // arm = new Arm();
    
    // The first argument is the root container
    // The second argument is whether logging and config should be given separate tabs
    Logger.configureLoggingAndConfig(this, false);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Update the odometry for the swerve drive
    swerve.periodic();

    // Update the logger for shuffleboard
    Logger.updateEntries();

  }
  
  @Override
  public void disabledInit() {

    // Set the swerve drive to coast mode
    

  }

  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {

    SwerveTrajectory.resetTrajectoryStatus();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    autoWaypoints.autoPeriodic();
    
    SwerveTrajectory.PathPlannerRunner(autoWaypoints.squarePath, swerve, swerve.getOdometry(), swerve.getPose().getRotation());

  }

  @Override
  public void teleopInit() {
    
    swerve.resetEncoders();
  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {

    double leftY = driver.getLeftX();
    double leftX = driver.getLeftY();
    double rightX = driver.getRightX();
    double deadZone = 0.5;
    
    if (Math.abs(leftY) < deadZone) {
      leftY = 0;
    }
    if (Math.abs(leftX) < deadZone) {
      leftX = 0;
    }
    if (Math.abs(rightX) < deadZone) {
      rightX = 0;
    }
    
    if (driver.getLeftBumper()) {
      swerve.setX();
    }
    else
    {
      // Drive the robot  
      //           SpeedX SpeedY Rotation
      swerve.drive(leftX, leftY, rightX, true);
    }
    // arm.drive(operator.getRightX(), operator.getRightY());

  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}