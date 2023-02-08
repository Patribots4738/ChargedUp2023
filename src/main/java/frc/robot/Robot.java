// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import hardware.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // The robot's subsystems and commands are defined here...
  /* ExampleSubsystem exampleSubsystem; */

  NewSwerve swerve;

  XboxController driver;
  XboxController operator;

  @Override
  public void robotInit() {
    swerve = new NewSwerve();
    driver = new XboxController(0);
    operator = new XboxController(1);
  }

  @Override
  public void robotPeriodic() {
    swerve.periodic();
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {

    if(!driver.getAButton())
    {
      swerve.drive(
              new Translation2d(
                      driver.getLeftX(),
                      driver.getLeftY()),
              new Translation2d(
                      driver.getRightX(),
                      driver.getRightY()
              ),
              true);
    }
  }
}
