// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import debug.*;
import edu.wpi.first.wpilibj.DriverStation;
import hardware.*;
import math.ArmCalculations;
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

    // Arm arm;

    Debug debug;

    ArmCalculations armCalcuations;


    @Override
    public void robotInit() {
        // Instantiate our Robot. This acts as a dictionary for all of our subsystems

        // Debug class for ShuffleBoard
        debug = new Debug();

        // Drivetrain instantiation
        swerve = new Swerve();

        driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
        operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

        // arm = new Arm();

        armCalcuations = new ArmCalculations();

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

        // autoSegmentedWaypoints.init(swerve, arm);
        SwerveTrajectory.resetTrajectoryStatus();

    }

    @Override
    public void autonomousPeriodic() {

        autoSegmentedWaypoints.periodic();
        // arm.periodic();

    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        // arm.periodic();

        // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
        // This is to ensure that the up direction on the joystick is positive inputs
        double driverLeftX    = MathUtil.applyDeadband(driver.getLeftX()   , OIConstants.DRIVER_DEADBAND);
        double driverLeftY    = MathUtil.applyDeadband(-driver.getLeftY()   , OIConstants.DRIVER_DEADBAND);
        double driverRightX   = MathUtil.applyDeadband(driver.getRightX()  , OIConstants.DRIVER_DEADBAND);
        double driverRightY   = MathUtil.applyDeadband(driver.getRightY()  , OIConstants.DRIVER_DEADBAND);

        double operatorLeftX  = MathUtil.applyDeadband(operator.getLeftX() , OIConstants.DRIVER_DEADBAND);
        double operatorLeftY  = MathUtil.applyDeadband(operator.getLeftY() , OIConstants.DRIVER_DEADBAND);
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
        // if (operator.getLeftStickButtonPressed()) {
        //     arm.toggleOperatorOverride();
        // }
        // if (arm.getOperatorOverride()) {
        //     arm.drive(new Translation2d(operatorLeftAxis.getX(), operatorLeftAxis.getY()));
        // }
        // else if (operator.getRightBumperPressed()) {
        //     arm.setArmIndex(arm.getArmIndex() + 1);
        // }
        // else if (operator.getLeftBumperPressed()) {
        //     arm.setArmIndex(arm.getArmIndex() - 1);
        // }

    }

    @Override
    public void testInit() {}


    @Override
    public void testPeriodic() {}
}
