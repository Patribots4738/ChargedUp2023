// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import hardware.*;
import math.ArmCalcuations;
import math.Constants.*;
import auto.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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

  // Swerve swerve;

  XboxController driver;
  // XboxController operator;
  
  AutoWaypoints autoWaypoints;

  Arm arm;
  
  HolonomicDriveController autoController; 

  Trajectory trajectory;

  Debug debug;

  ArmCalcuations armCalcuations = new ArmCalcuations();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
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
    // swerve = new Swerve();
    // // Zero the IMU for field-oriented driving 
    // swerve.resetEncoders();
    // swerve.zeroHeading();
    // swerve.setBrakeMode();

    // Setup controllers
    driver = new XboxController(OIConstants.kDriverControllerPort);
    // operator = new XboxController(OIConstants.kOperatorControllerPort);

    // Arm Instantiation
    arm = new Arm();
    
    
    // AutoWaypoints Instantiation
    autoWaypoints = new AutoWaypoints();

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

    // autoWaypoints.init(swerve); 
    // // swerve.setCoastMode();
    // SwerveTrajectory.resetTrajectoryStatus();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // autoWaypoints.autoPeriodic();
    // SwerveTrajectory.PathPlannerRunner(autoWaypoints.squarePath, swerve, swerve.getOdometry(), swerve.getPose().getRotation());


  }

  @Override
  public void teleopInit() {
    
    // swerve.resetEncoders();
    // swerve.setBrakeMode();
    arm.resetEncoders();

  }

  /** This function is called periodically during operator control. */
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
      // swerve.drive(leftY*0.25, leftX*0.25, rightX*0.25, true);
    // }


    // Notice that the input of the lower arm pos is 
    // revolutions / 5 because we want 360/5 = 72 degrees in both directions
    if (driver.getLeftBumper()) {
      
      arm.drive(leftX, leftY);
      
      // arm.setLowerArmPosition(0);
      // arm.setUpperArmPosition(leftX/5);
      // Yummy debug makes me giddy
      double lowerAngle = armCalcuations.getLowerAngle(leftX, leftY);
      System.out.println(
                "LeftX: "  + String.format("%.3f", leftX) +
               " LeftY: " + String.format("%.3f", leftY) +
               " Q1: "    + String.format("%.3f", Units.radiansToDegrees(armCalcuations.getUpperAngle(leftX, leftY, lowerAngle))) +
               " Q2: "    + String.format("%.3f", Units.radiansToDegrees(lowerAngle)-90));

    }

  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // int targetID = -1;
    // double x = -1;
    // double y = -1;
    // double pitch = -1;
    // double yaw = -1;

    //  var result = camera.getLatestResult();

    //  if (result.hasTargets()){

    //     PhotonTrackedTarget bestTarget = result.getBestTarget();

    //     if (targetID == 1){

    //       targetID = bestTarget.getFiducialId();
    //       x = bestTarget.getBestCameraToTarget().getTranslation().getX();
    //       y = bestTarget.getBestCameraToTarget().getTranslation().getY();
    //       pitch = bestTarget.getPitch();
    //       yaw = bestTarget.getYaw();
        
    //     }
          
    //   }
      
    //   SmartDashboard.putNumber("X", x);
    //   SmartDashboard.putNumber("Y", y);
    //   SmartDashboard.putNumber("Pitch", pitch); 
    //   SmartDashboard.putNumber("Yaw", yaw);

    //   // if(result.hasTargets()) {
    //   //   double x = result.getBestTarget().getBestCameraToTarget().getTranslation().getX();
    //   //   double y = result.getBestTarget().getBestCameraToTarget().getTranslation().getY();
    //   // }
  }
}