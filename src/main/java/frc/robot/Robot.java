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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // The robot's subsystems and commands are defined here...

    Swerve swerve;

    XboxController driver;
    XboxController operator;

    AutoSegmentedWaypoints autoSegmentedWaypoints;

    Arm arm;

    Debug debug;

    ArmCalcuations armCalcuations = new ArmCalcuations();


    @Override
    public void robotInit() {
        // Instantiate our Robot. This acts as a dictionary for all of our subsystems

        // Debug class for ShuffleBoard
        debug = new Debug();
        debug.debugInit();

        // Drivetrain instantiation
        swerve = new Swerve();
        // Zero the IMU for field-oriented driving
        swerve.resetEncoders();
        swerve.zeroHeading();

        swerve.setBrakeMode();

        driver = new XboxController(OIConstants.kDriverControllerPort);
        operator = new XboxController(OIConstants.kOperatorControllerPort);

        arm = new Arm();
        arm.resetEncoders();
        arm.setBrakeMode();

        autoSegmentedWaypoints = new AutoSegmentedWaypoints();
        autoSegmentedWaypoints.loadAutoPaths();

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

        autoSegmentedWaypoints.init(swerve, arm);
        SwerveTrajectory.resetTrajectoryStatus();

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
        double driverLeftX = -MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriverDeadband);
        double driverLeftY = -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriverDeadband);
        double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriverDeadband);
        double driverRightY = -MathUtil.applyDeadband(driver.getRightY(), OIConstants.kDriverDeadband);
        
        double operatorLeftX  = MathUtil.applyDeadband(operator.getLeftX(), OIConstants.kDriverDeadband);
        double operatorLeftY  = MathUtil.applyDeadband(operator.getLeftY(), OIConstants.kDriverDeadband);
        double operatorRightX = MathUtil.applyDeadband(operator.getRightX(), OIConstants.kDriverDeadband);
        double operatorRightY = MathUtil.applyDeadband(operator.getRightY(), OIConstants.kDriverDeadband);

        Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);
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

        if (driver.getRightBumperPressed()) {
            arm.setArmIndex(arm.getArmIndex() + 1);
        }
        else if (driver.getLeftBumperPressed()) {
            arm.setArmIndex(arm.getArmIndex() - 1);
        }
        
        // Toggle the operator override when the operator's left stick is pressed
        if (operator.getLeftStickButtonPressed()) {
          arm.toggleOperatorOverride();
        }
        if (arm.getOperatorOverride()) {
          arm.drive(new Translation2d(operatorLeftAxis.getX(), operatorLeftAxis.getY()));
        }
        else if (driver.getRightBumperPressed()) {
            arm.setArmIndex(arm.getArmIndex() + 1);
        }
        else if (driver.getLeftBumperPressed()) {
            arm.setArmIndex(arm.getArmIndex() - 1);
        }


    }

    @Override
    public void testInit() {}


    @Override
    public void testPeriodic() {}
}
