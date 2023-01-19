// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;

import debug.Debug;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import hardware.*;
import math.*;
import math.Constants.*;
import auto.*;

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
  
  Auto auto;
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
    // Instantiate our Robot. This will perform all our button bindings, and put our
    
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
    
    // Auto instantiation
    auto = new Auto();
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

  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {

    autoPath = new TestPath(swerve);

    // Here, our rotation profile constraints were a max velocity
    // of 1 rotation per second and a max acceleration of 180 degrees
    // per second squared.
    autoController = auto.getAutoController(Constants.AutoConstants.kXCorrectionP, Constants.AutoConstants.kXCorrectionI, Constants.AutoConstants.kXCorrectionD, 
      Constants.AutoConstants.kYCorrectionP, Constants.AutoConstants.kYCorrectionI, Constants.AutoConstants.kYCorrectionD,
      Constants.AutoConstants.kRotationCorrectionP, Constants.AutoConstants.kRotationCorrectionI, Constants.AutoConstants.kRotationCorrectionD,
      Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // autoController = new HolonomicDriveController(
    //   new PIDController(kP.getDouble(1.0), kI.getDouble(0.0), kD.getDouble(0.0)),
    //   new PIDController(kP.getDouble(1.0), kI.getDouble(0.0), kD.getDouble(0.0)),
    //   new ProfiledPIDController(kP2.getDouble(1.0), kI2.getDouble(0.0), kD2.getDouble(0.0),
    //     new TrapezoidProfile.Constraints(1, 2)));
    Trajectory trajectory = autoPath.getPath();
    // Sample the trajectory at half of the length
    List<State> goal = trajectory.getStates();

    // Get the adjusted speeds. Here, we want the robot to be facing
    // 45 degrees (in the field-relative coordinate system).
    ChassisSpeeds adjustedSpeeds = autoController.calculate(
    swerve.getPose(), goal.get(auto.getAutoStates()), goal.get(auto.getAutoStates()).poseMeters.getRotation());
    
    
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);
    
    swerve.setModuleStates(moduleStates);

    debug.debugPeriodic(adjustedSpeeds.vyMetersPerSecond);
    System.out.println("Swerve pos: " + swerve.getPose().getX());    

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
  public void testInit() {
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}