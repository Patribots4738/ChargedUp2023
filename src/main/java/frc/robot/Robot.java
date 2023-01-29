// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import hardware.*;
import math.ArmCalcuations;
import math.OICalc;
import math.Constants.*;
import auto.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
  // XboxController operator;
  
  AutoSegmentedWaypoints autoSegmentedWaypoints;

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
    swerve = new Swerve();
    // // Zero the IMU for field-oriented driving 
    swerve.resetEncoders();
    swerve.zeroHeading();
    swerve.setBrakeMode();

    // Setup controllers
    driver = new XboxController(OIConstants.kDriverControllerPort);
    // operator = new XboxController(OIConstants.kOperatorControllerPort);

    // Arm Instantiation
    arm = new Arm();
    
    
    // AutoSegmentedWaypoints Instantiation
    autoSegmentedWaypoints = new AutoSegmentedWaypoints();
    autoSegmentedWaypoints.loadAutoPaths();

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
    // swerve.setCoastMode();
    
  }

  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {

    autoSegmentedWaypoints.init(swerve, arm); 
    // swerve.setCoastMode();
    SwerveTrajectory.resetTrajectoryStatus();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    autoSegmentedWaypoints.autoPeriodic();
    // SwerveTrajectory.PathPlannerRunner(autoSegmentedWaypoints.squarePath, swerve, swerve.getOdometry(), swerve.getPose().getRotation());


  }

  @Override
  public void teleopInit() {
    
    // swerve.resetEncoders();
    // arm.resetEncoders();

    swerve.setBrakeMode();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {

    // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
    // This is to ensure that the up direction on the joystick is positive inputs
    double driverLeftX  =  MathUtil.applyDeadband(driver.getLeftX(),  OIConstants.kDriverDeadband);
    double driverLeftY  = -MathUtil.applyDeadband(driver.getLeftY(),  OIConstants.kDriverDeadband);
    double driverRightX =  MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriverDeadband);
    double driverRightY = -MathUtil.applyDeadband(driver.getRightY(), OIConstants.kDriverDeadband);

    Translation2d armInputs = OICalc.toCircle(driverLeftX, 0.75);

    // if (driver.getRightBumper()) {
    //   swerve.setX();
    // }
    // else
    // {
    //   Drive the robot  
    //   swerve.drive(SpeedX, SpeedY, Rotation, Field_Oriented);
    //   swerve.drive(leftY*0.25, leftX*0.25, rightX*0.25, true);
    // }

    if (driver.getLeftBumper()) { 
      
      // arm.drive(armInputs.getX(), armInputs.getY());
      arm.setUpperArmPosition(driverLeftX/5);
      arm.setLowerArmPosition(0);
      // Yummy debug makes me giddy
      double upperAngle = armCalcuations.getUpperAngle(armInputs.getX(), armInputs.getY());
      System.out.println(
                "LeftX: "  + String.format("%.3f", armInputs.getX()) +
               " LeftY: " + String.format("%.3f", armInputs.getY()) +
               " Q1: "    + String.format("%.3f", Units.radiansToDegrees(armCalcuations.getLowerAngle(armInputs.getX(), armInputs.getY(), upperAngle))) +
               " Q2: "    + String.format("%.3f", Units.radiansToDegrees(upperAngle)-90));

    
    
    }
    else if (driver.getBButtonPressed()) {
      arm.setArmIndex(1);
    }
    else if (driver.getXButtonPressed()) {
      arm.setArmIndex(-1);
    }
    
  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}