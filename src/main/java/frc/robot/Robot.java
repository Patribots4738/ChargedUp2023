// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import hardware.*;
import math.ArmCalcuations;
import math.Constants;
import math.Constants.*;
import auto.*;
import subsystems.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Logger;

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

  // Arm arm;
  
  HolonomicDriveController autoController; 

  Trajectory trajectory;

  Debug debug;

  // ArmCalcuations armCalcuations = new ArmCalcuations();

  Vision vision = new Vision();

  Boolean isAlligned = false;

  Boolean isHorizontallyAlligned = false;

  HolonomicDriveController HDC;

  Pose2d aprilPos;

  AutoAllignment autoAllignment;

  @Override
  public void robotInit() {

    // Instantiate our Robot. This acts as a dictionary for all of our subsystems
    
    // Debug class so we can use shuffleboard
    debug = new Debug();
    debug.debugInit();


    /*
      For swerve drive, the following is the order of the motors
      odd CAN IDs drive the robot
      even CAN IDs are the turning motors
     */
    // Drivetrain instantiation
    swerve = new Swerve();
    autoAllignment = new AutoAllignment(swerve);
    // // Zero the IMU for field-oriented driving 
    swerve.resetEncoders();
    swerve.zeroHeading();
    swerve.setBrakeMode();

    // Setup controllers
    driver = new XboxController(OIConstants.kDriverControllerPort);
    operator = new XboxController(OIConstants.kOperatorControllerPort);

    // Arm Instantiation
    // arm = new Arm();
    
    
    // AutoWaypoints Instantiation
    autoWaypoints = new AutoWaypoints();

    // The first argument is the root container
    // The second argument is whether logging and config should be given separate tabs
    Logger.configureLoggingAndConfig(this, false);

    SwerveTrajectory.resetTrajectoryStatus();
    autoWaypoints.init(swerve);

  }

  @Override
  public void robotPeriodic() {

    // Update the odometry for the swerve drive
    // swerve.periodic();

    // Update the logger for shuffleboard
    Logger.updateEntries();

  }
  
  @Override
  public void disabledInit() {

    // Set the swerve drive to coast mode
    // swerve.setCoastMode();
    
  }

  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {
    autoWaypoints.init(swerve);
  }
  @Override
  public void autonomousPeriodic() {

    autoWaypoints.autoPeriodic();
    SwerveTrajectory.PathPlannerRunner(autoWaypoints.testTraj, swerve, swerve.getOdometry(), swerve.getPose().getRotation());

  }

  @Override
  public void teleopInit() {
    
    swerve.resetEncoders();
    swerve.setBrakeMode();
    // arm.resetEncoders();
    HDC = SwerveTrajectory.getHDC();

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    swerve.resetEncoders();
    swerve.setBrakeMode();
    HDC = SwerveTrajectory.getHDC();
  }

  @Override
  public void testPeriodic() {

    double driverLeftX = MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriverDeadband);
    double driverLeftY = -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriverDeadband);
    double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriverDeadband);
    double driverRightY = -MathUtil.applyDeadband(driver.getRightY(), OIConstants.kDriverDeadband);
    
    if (driver.getLeftBumper()) {
      swerve.drive(driverLeftX * 0.25, driverLeftY * 0.25, driverRightX * 0.25, true);
    }

    // Use the A button to activate the allignment process
    if (operator.getAButton()) {

      // Run the vision calculations and get the most visible tag
      vision.pereodic();

      // Make sure that the camera has tags in view
      if (vision.hasTargets()) {

        int tagID = vision.getTagID();
        autoAllignment.moveToTag(tagID, HDC, autoWaypoints);

      }
      // Make the controller buzz if there are no targets in view
      else {
        operator.setRumble(RumbleType.kLeftRumble, 0.);
        isHorizontallyAlligned = false;
      }
    }
  }
}
