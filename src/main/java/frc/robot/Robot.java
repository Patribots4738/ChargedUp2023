// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;

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
import frc.robot.autos.*;
import hardware.*;
import math.*;
import math.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // The robot's subsystems and commands are defined here...
  // ExampleSubsystem exampleSubsystem; 
  CANSparkMax motor;
  Swerve swerve;
  XboxController driver;
  XboxController operator;
  MAXSwerveModule module;
  // Arm arm;
  HolonomicDriveController autoController;
  Trajectory trajectory;
  TestPath autoPath;

  GenericEntry kP; GenericEntry kI; GenericEntry kD; GenericEntry kP2; GenericEntry kI2; GenericEntry kD2; GenericEntry m_speed;
  int autoStates = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our Robot. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    swerve = new Swerve();

    driver = new XboxController(OIConstants.kDriverControllerPort);
    operator = new XboxController(OIConstants.kOperatorControllerPort);

    // swerve.resetOdometry(new Pose2d(0.0, 0.0, 0));

    // module = new MAXSwerveModule(1, 2, 0)
    
    
    // arm = new Arm();

          
    kP = Shuffleboard.getTab("Drive")
      .add("P", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 10)) // specify widget properties here
      .getEntry();

    kI = Shuffleboard.getTab("Drive")
      .add("I", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    kD = Shuffleboard.getTab("Drive")
      .add("D", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();
   
    kP2 = Shuffleboard.getTab("Turn")
      .add("P", 0.22)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -5, "max", 5)) // specify widget properties here
      .getEntry();

    kI2 = Shuffleboard.getTab("Turn")
      .add("I", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    kD2 = Shuffleboard.getTab("Turn")
      .add("D", 0.74)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    m_speed = Shuffleboard.getTab("Turn")
      .add("Speed", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
      .getEntry();

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
    autoController = new HolonomicDriveController(
      new PIDController(kP.getDouble(1.0), kI.getDouble(0.0), kD.getDouble(0.0)),
      new PIDController(kP.getDouble(1.0), kI.getDouble(0.0), kD.getDouble(0.0)),
      //  new ProfiledPIDController(kP2.getDouble(0.22), kI2.getDouble(0.0), kD2.getDouble(0.74),
        new ProfiledPIDController(kP2.getDouble(3.596), kI2.getDouble(0.0), kD2.getDouble(0),
        // new TrapezoidProfile.Constraints(6.28, 1)));
        new TrapezoidProfile.Constraints(0.5, 0.5)));
    
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
    swerve.getPose(), goal.get(autoStates), goal.get(autoStates).poseMeters.getRotation());
    
    

    System.out.println("Swerve pos: " + swerve.getPose().getX());    
    
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

    m_speed.setDouble(adjustedSpeeds.vyMetersPerSecond);
    
    swerve.setModuleStates(moduleStates);

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