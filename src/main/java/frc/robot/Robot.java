// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import hardware.*;
import subsystems.PhotonCameraPose;
import math.Constants.*;
import auto.*;
import subsystems.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import math.OICalc;
import edu.wpi.first.math.geometry.Translation2d;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // The robot's subsystems and commands are defined here...

  Swerve swerve;
  SwerveTrajectory swerveTrajectory;

  XboxController driver;
  XboxController operator;

  AutoSegmentedWaypoints autoSegmentedWaypoints;

  Arm arm;

  Debug debug;

  Vision vision;

  AutoAlignment autoAlignment;

  PhotonCameraPose photonPose;

  @Override
  public void robotInit() {

        // Instantiate our Robot. This acts as a dictionary for all of our subsystems

        // Debug class for Shuffleboard
        debug = new Debug();
        debug.debugInit();

        /*
          For swerve drive, the following is the order of the motors
          odd CAN IDs drive the robot
          even CAN IDs are the turning motors
         */
        // Drivetrain instantiation
        swerve = new Swerve();
        autoAlignment = new AutoAlignment(swerve);
        // // Zero the IMU for field-oriented driving
        swerve.resetEncoders();
        swerve.zeroHeading();
        swerve.setBrakeMode();

        driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
        operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

        arm = new Arm();
        arm.resetEncoders();
        arm.setBrakeMode();

        autoSegmentedWaypoints = new AutoSegmentedWaypoints(swerve, arm);
        autoSegmentedWaypoints.loadAutoPaths();

        swerveTrajectory = new SwerveTrajectory();
        swerveTrajectory.resetTrajectoryStatus();

        vision = new Vision();

        // Configure the logger for shuffleboard
        Logger.configureLoggingAndConfig(this, false);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Used for items like diagnostics
     * ran during disabled, autonomous, teleoperated and test. :D
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        swerve.periodic();
        // arm.periodic();

        Logger.updateEntries();

  }

  @Override
  public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {

        swerveTrajectory.resetTrajectoryStatus();

    }

    @Override
    public void autonomousPeriodic() {

        autoSegmentedWaypoints.periodic();
        arm.periodic();

    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        arm.periodic();

        // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
        // This is to ensure that the up direction on the joystick is positive inputs
        double driverLeftX  = MathUtil.applyDeadband(driver.getLeftX(), OIConstants.DRIVER_DEADBAND);
        double driverLeftY  = MathUtil.applyDeadband(driver.getLeftY(), OIConstants.DRIVER_DEADBAND);
        double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADBAND);
        double driverRightY = MathUtil.applyDeadband(driver.getRightY(), OIConstants.DRIVER_DEADBAND);

        double operatorLeftX  = MathUtil.applyDeadband(operator.getLeftX(), OIConstants.DRIVER_DEADBAND);
        double operatorLeftY  = MathUtil.applyDeadband(operator.getLeftY(), OIConstants.DRIVER_DEADBAND);
        double operatorRightX = MathUtil.applyDeadband(operator.getRightX(), OIConstants.DRIVER_DEADBAND);
        double operatorRightY = MathUtil.applyDeadband(operator.getRightY(), OIConstants.DRIVER_DEADBAND);

        Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);
        // If we are on blue alliance, flip the driverLeftAxis
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            driverLeftAxis = driverLeftAxis.unaryMinus();
        }

        Translation2d operatorLeftAxis = OICalc.toCircle(operatorLeftX, operatorLeftY);

        if (driver.getRightBumper()) {
            swerve.setX();
        } else {
            //              SpeedX,               SpeedY,              Rotation,    Field_Oriented
            swerve.drive(driverLeftAxis.getX(), driverLeftAxis.getY(), driverRightX, true);
        }

        // Toggle the speed to be 10% of max speed when the driver's left stick is pressed
        if (driver.getLeftStickButtonPressed()) {
            swerve.toggleSpeed();
        }

        // Toggle the operator override when the operator's left stick is pressed
        if (operator.getLeftStickButtonPressed()) {
            arm.toggleOperatorOverride();
        }
        if (arm.getOperatorOverride()) {
            arm.drive(new Translation2d(operatorLeftAxis.getX(), operatorLeftAxis.getY()));
        }
        else if (operator.getRightBumperPressed()) {
            arm.setArmIndex(arm.getArmIndex() + 1);
        }
        else if (operator.getLeftBumperPressed()) {
            arm.setArmIndex(arm.getArmIndex() - 1);
        }

    }

  @Override
  public void testInit() {
    swerve.resetEncoders();
    swerve.setBrakeMode();
    swerveTrajectory.resetTrajectoryStatus();

  }

  @Override
  public void testPeriodic() {

    double driverLeftX  = MathUtil.applyDeadband(driver.getLeftX(), OIConstants.DRIVER_DEADBAND);
    double driverLeftY  = MathUtil.applyDeadband(driver.getLeftY(), OIConstants.DRIVER_DEADBAND);
    double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADBAND);

    Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);
    // If we are on blue alliance, flip the driverLeftAxis

    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      driverLeftAxis = driverLeftAxis.unaryMinus();
    }

    // Use the A button to activate the alignment process
    if (driver.getAButton()) {

      // Run the vision calculations and get the most visible tag
      vision.periodic();

      // Make sure that the camera has tags in view
      if (vision.hasTargets()) {

        autoAlignment.setTagID(vision.getTagID());

        if (driver.getLeftBumperPressed()) {

          System.out.println("Swerve Before Align: " + swerve.getPose() + "\n\n");
          System.out.println("Distance from april to bot: " + vision.getTransform().getTranslation() + " " + vision.getTransform().getRotation().getZ() + "\n\n");

          // Use the new vision system to get odometry here...
          // May be swerve.updateOdometry...
          //autoAlignment.calibrateOdometry(vision.getTransform());
          swerveTrajectory.resetTrajectoryStatus();

          System.out.println("Swerve After Align: " + swerve.getPose() + "\n\n");

        }
      }

      autoAlignment.moveToTag();

      if (driver.getRightBumper()) {
        switch (driver.getPOV()) {
          // Not clicked
          case -1:
            break;

          // Clicking up
          case 0:
            arm.setArmIndex(arm.getArmIndex() + 1);
            break;

          // Clicking down
          case 180:
            arm.setArmIndex(arm.getArmIndex() - 1);
            break;

          // Clicking left
          case 270:
            autoAlignment.setConeOffset(autoAlignment.getConeOffset() - 1);
            break;

          // Clicking right
          case 90:
            autoAlignment.setConeOffset(autoAlignment.getConeOffset() + 1);
            break;
        }
      }

    } else {

      swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), driverRightX*.25, true);

    }
  }
}
