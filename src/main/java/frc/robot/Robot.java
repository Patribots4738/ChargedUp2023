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
  public void teleopPeriodic() 
  {

    double leftX = driver.getLeftX();
    double leftY = -driver.getLeftY();
    double rightX = driver.getRightX();
    double deadZone = 0.15;
    
    if (Math.abs(leftY) < deadZone) {
      leftY = 0;
    }
    if (Math.abs(leftX) < deadZone) {
      leftX = 0;
    }
    if (Math.abs(rightX) < deadZone) {
      rightX = 0;
    }
    
    // if (driver.getLeftBumper()) {
    //   swerve.setX();
    // }
    // else
    // {
      // Drive the robot  
      //           SpeedX SpeedY Rotation
      swerve.drive(leftX*0.25, leftY*0.25, rightX*0.25, true);
    // }


    // Notice that the input of the lower arm pos is 
    // revolutions / 5 because we want 360/5 = 72 degrees in both directions
    // if (driver.getLeftBumper()) {
      
    //   // arm.drive(leftX, leftY);
      
    //   // arm.setLowerArmPosition(0);
    //   // arm.setUpperArmPosition(leftX/5);
    //   // Yummy debug makes me giddy
    //   double lowerAngle = armCalcuations.getLowerAngle(leftX, leftY);
    //   System.out.println(
    //             "LeftX: "  + String.format("%.3f", leftX) +
    //            " LeftY: " + String.format("%.3f", leftY) +
    //            " Q1: "    + String.format("%.3f", Units.radiansToDegrees(armCalcuations.getUpperAngle(leftX, leftY, lowerAngle))) +
    //            " Q2: "    + String.format("%.3f", Units.radiansToDegrees(lowerAngle)-90));

    // }

  }

  @Override
  public void testInit() {
    swerve.resetEncoders();
    swerve.setBrakeMode();
    HDC = SwerveTrajectory.getHDC();
  }

  @Override
  public void testPeriodic() {
    swerve.periodic();
    vision.pereodic();

    double leftX = driver.getLeftX();
    double leftY = driver.getLeftY();
    double rightX = driver.getRightX();
    double deadZone = 0.15;

    swerve.periodic();
    
    if (!driver.getAButton()){
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
        swerve.drive(-leftX*0.25, - leftY*0.25, rightX*0.25, true);
      }
    }
    else {

      if (driver.getAButton()){
        if (driver.getLeftBumperPressed()){
          System.out.println("Callibrate");
          autoAllignment.callibrateOdometry(vision.getPose(), vision.getTagID());
        }

        else if (driver.getRightBumper()){
          System.out.println("Move to tag");
          autoAllignment.moveToTag(vision.getTagID(), HDC, autoWaypoints);
        }
      }        
    }
    
    
    
  }
}
