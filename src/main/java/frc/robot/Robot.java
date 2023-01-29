// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import hardware.*;
import math.ArmCalcuations;
import math.Constants;
import math.OICalc;
import math.Constants.*;
import auto.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

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
  
  AutoSegmentedWaypoints autoSegmentedWaypoints;

  Arm arm;
  
  HolonomicDriveController autoController; 

  Trajectory trajectory;

  Debug debug;

  ArmCalcuations armCalcuations = new ArmCalcuations();
  

  Vision vision = new Vision();

  Boolean isHorizontallyAlligned = false;

  HolonomicDriveController HDC = SwerveTrajectory.getHDC();

  Pose2d aprilPos;


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
    operator = new XboxController(OIConstants.kOperatorControllerPort);

    // Arm Instantiation
    // arm = new Arm();
    
    
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
      
      // arm.drive(leftX, leftsY);
      
      // arm.setLowerArmPosition(0);
      // arm.setUpperArmPosition(leftX/5);
      // Yummy debug makes me giddy
      double upperAngle = armCalcuations.getUpperAngle(armInputs.getX(), armInputs.getY());
      System.out.println(
                "LeftX: "  + String.format("%.3f", armInputs.getX()) +
               " LeftY: " + String.format("%.3f", armInputs.getY()) +
               " Q1: "    + String.format("%.3f", Units.radiansToDegrees(armCalcuations.getLowerAngle(armInputs.getX(), armInputs.getY(), upperAngle))) +
               " Q2: "    + String.format("%.3f", Units.radiansToDegrees(upperAngle)-90));

    
    
    }
    
  }

  @Override
  public void testInit() {
    swerve.resetEncoders();
    swerve.setBrakeMode();
  }

  
  @Override
  public void testPeriodic() {

    double leftX = driver.getLeftX();
    double leftY = driver.getLeftY();
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
    
    if (driver.getLeftBumper()) {
      swerve.setX();
    }
    else
    {
    // Drive the robot  
    //           SpeedX SpeedY Rotation
      swerve.drive(leftX*0.25, leftY*0.25, rightX*0.25, true);
    }

     // Use the A button to activate the allignment process
     if (operator.getAButton()){
          
      // Run the vision calculations and get the most visible tag
      vision.pereodic();

      // Make sure that the camera has tags in view
      if (vision.hasTargets()){

        aprilPos = vision.getPose();

        // Get all the data we need for allignment
        double yaw = vision.getYaw();
        double x = vision.getX();
        int tagID = vision.getTagID();

        // Direction is Left: -1, Right: 1, Default: 0
        int direction = 0;

        if (yaw > vision.rotDeadzone){
          // Rotate the robot in the negative yaw direction
          swerve.drive(0, 0, vision.allignmentRotSpeed, true);
        }
        
        else if (yaw < -vision.rotDeadzone){
          // Rotate the robot in the positive yaw direction
          swerve.drive(0, 0, -vision.allignmentRotSpeed, true);
        }

        /**
         * The operator has two buttons they can press, 
         * one moves the robot to the left if there's a valid position to the left,
         * and the other does the same for the right.
         * 
         * The distances for these positions are set as an array tagPositions,
         * which follows format:
         * {1:[leftDist, rightDist], 2:[leftDist, rightDist], ...}
        */

        /**
         *  A visual representation of the apriltag positions
         *  / --------------------------------------------- \ 
         *  5                      |                        4
         *  |                      |                        |
         *  |                      |                        |
         *  6                      |                        3
         *  |                      |                        |
         *  7                      |                        2
         *  |                      |                        |
         *  8                      |                        1
         *  \ --------------------------------------------- /
         */
        
        
        else {
        
          // Check the horizontal allignment of the robot relative to the tag
          // "Horizontal", "left", and "right" are all relative to the robot in this circumstance
          if (x > vision.xDeadZone){
            // Move the robot left at x speed
            swerve.drive(0, vision.allignmentSpeed, 0, true);
          }
          else if (x < -vision.xDeadZone){
            // Move the robot right at x speed
            swerve.drive(0, -vision.allignmentSpeed, 0, true);
          }
          else {
            // Stop the robot
            swerve.drive(0, 0, 0, true);
            isHorizontallyAlligned = true;
            
          }
        }

        // Make the controller buzz if the robot is alligned horizontally
        if (isHorizontallyAlligned){
          operator.setRumble(RumbleType.kRightRumble, 0.5);
        }
        else {
          operator.setRumble(RumbleType.kRightRumble, 0);

          // ChassisSpeeds _speeds = HDC.calculate(
          //   // Current pose
          //   (swerve.getOdometry().getPoseMeters()), 
          //   // Desired pose, which is the 
          //   // our pose on the X since we don't care about how far we are from it,
          //   // and the april pose + the offset of the cone on the Y
          //   // and the rotation of the april tag, to align us with the field
          //   (new Pose2d(swerve.getOdometry().getPoseMeters().getX(), 
          //   vision.getY() + Constants.VisionConstants.kConeOffsetMeters, 
          //   vision.getPose().getRotation())),
          //   // Desired velocity
          //   Constants.VisionConstants.kDesiredVelocityMetersPerSecond,
          //   // Desired rotation (the field)
          //   (aprilPos.getRotation()));

          // // if (Math.abs(_speeds.vyMetersPerSecond) < 0.01){
          // //   isHorizontallyAlligned = true;
          // // }

          // swerve.drive(leftX*0.25,
          // _speeds.vyMetersPerSecond*.25, 
          // _speeds.omegaRadiansPerSecond*.25,false);
          
        }


      }

      // Make the controller buzz if there are no targets in view
      else {
        operator.setRumble(RumbleType.kLeftRumble, 0.);
        isHorizontallyAlligned = false;
      }
    }

    // Set isAlligned to false if the A button is not pressed
    else {
      isHorizontallyAlligned = false;
      operator.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
