// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import hardware.*;
import math.ArmCalcuations;
import math.Constants.*;

import com.revrobotics.CANSparkMax;

import auto.*;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // The robot's subsystems and commands are defined here...
  /* ExampleSubsystem exampleSubsystem; */

  Swerve swerve;

  XboxController driver;
  XboxController operator;
  
  AutoWaypoints autoWaypoints;

  Arm arm;
  
  HolonomicDriveController autoController; 

  Trajectory trajectory;

  Debug debug;

  ArmCalcuations armCalcuations = new ArmCalcuations();

  Vision vision = new Vision();

  Boolean isHorizontallyAlligned = false;

  HolonomicDriveController HDC = SwerveTrajectory.getHDC();

  Pose2d aprilPos;

  CANSparkMax _testMotor;


  @Override
  public void robotInit() {
    // Setup controllers
    driver = new XboxController(OIConstants.kDriverControllerPort);
    _testMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
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
    if(driver.getAButtonPressed())
    {
      _testMotor.set(0.5);
    }
    _testMotor.set(0);
  }
  
  @Override
  public void disabledInit() {

    
  }

  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {
  }
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
  }

  @Override
  public void testInit() {
  }

  
  @Override
  public void testPeriodic() {
  }
}
