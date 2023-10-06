package frc.robot;

import com.revrobotics.CANSparkMax;

import auto.AutoAlignment;
import auto.AutoPathStorage;
import auto.AutoSegmentedWaypoints;
import auto.SwerveTrajectory;
import calc.ArmCalculations;
import calc.Constants.AlignmentConstants;
import calc.Constants.AutoConstants;
import calc.Constants.DriveConstants;
import calc.Constants.LEDConstants;
import calc.Constants.NeoMotorConstants;
import calc.Constants.OIConstants;
import calc.Constants.PlacementConstants;
import calc.OICalc;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import hardware.ArduinoController;
import hardware.Arm;
import hardware.Claw;
import hardware.Swerve;
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
  AutoPathStorage autoPathStorage;
  
  Arm arm;
  Claw claw;

  ArduinoController arduinoController;
  ArmCalculations armCalculations;

  AutoAlignment autoAlignment;

  DriverUI driverUI = new DriverUI();

  public static Timer timer;

  @Override
  public void robotInit() {
      // Instantiate our Robot. This acts as a dictionary for all of our subsystems
      // Drivetrain instantiation
      swerve = new Swerve();

      driver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
      operator = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

      arm = new Arm();
      claw = new Claw();

      armCalculations = new ArmCalculations();
      autoAlignment = new AutoAlignment(swerve, claw);// Configure the logger for shuffleboard

      autoSegmentedWaypoints = new AutoSegmentedWaypoints(swerve, arm, claw, autoAlignment);
      autoPathStorage = new AutoPathStorage();

      arduinoController = new ArduinoController();

      timer = new Timer();

      Timer.delay(0.25);
      for (CANSparkMax neo : NeoMotorConstants.motors) {
        neo.burnFlash();
        Timer.delay(0.005);
      }
      Timer.delay(0.25);

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
        
    Logger.updateEntries();
    DriverUI.connected = DriverStation.isDSAttached() || DriverStation.isFMSAttached();
    arduinoController.periodic();
    claw.updateOutputCurrent();

  }

  @Override
  public void disabledInit() {
    // arm.setUpperArmCoastMode();
    claw.stopClaw();

    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);

    operator.setRumble(RumbleType.kLeftRumble, 0);
    operator.setRumble(RumbleType.kRightRumble, 0);
    
    arduinoController.setLEDState(LEDConstants.ARM_GREEN);

    swerve.setSpeedMultiplier(1);

    DriverUI.enabled = false;

    System.out.println(swerve.getPose().getTranslation());

  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.isDSAttached() || DriverStation.isFMSAttached()) {
        if (Math.abs(swerve.getPitch().getDegrees()) > 35) {
            arduinoController.setLEDState(LEDConstants.BELLY_PAN_FLASH_RED);
        }
        else {
            if (DriverStation.getAlliance() == Alliance.Blue) {
                arduinoController.setLEDState(LEDConstants.BELLY_PAN_BLUE);
            }
            else if (DriverStation.getAlliance() == Alliance.Red) {
                arduinoController.setLEDState(LEDConstants.BELLY_PAN_RED_ALLIANCE);
            }
        }
    }
  }

  @Override
  public void autonomousInit() {
    // Restart the timer to use at the end of auto
    timer.restart();
    DriveConstants.MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
    autoAlignment.setConeMode(true);
    arm.setBrakeMode();
    // initialize variables for auto
    autoSegmentedWaypoints.init();
    DriverUI.enabled = true;
    // We had a issue where we needed to redeploy before restarting auto
    // So this gives us an indicator to know if we have enabled yet
    // (i lost my mind not knowing if i redeployed or not) - alexander hamilton

    
    // future alegamnder here, that issue has since been fixed, 
    // but my dementia has not.
    // enjoy your boolboxes :)
    DriverUI.freshCode = false;
    SwerveTrajectory.resetTrajectoryStatus();
  }

  @Override
  public void autonomousPeriodic() {
    // Update odometry to know where we are on the field
    // and update our "position"
    swerve.periodic();
    claw.periodic();
    // System.out.printf("Time Left %.1f\n s", Timer.getMatchTime());
    // Force the claw to spit at the start of the match, for preload
    if (timer.get() < 0.08) {
      claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CONE);
      return;
    }
    // If we are in the normal duration of auto, 
    // let the arm move as normal
    else {
      arm.periodic();
    }

    // Have the claw outtake at the end of the match,
    // This is for a last second score
    if (timer.get() > 14.8) {
      claw.setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE);
    }
    // If we are in the last 100 ms of the match, set the wheels up
    // This is to prevent any charge pad sliding
    if (timer.get() > 14.9 && timer.get() < 15 && autoSegmentedWaypoints.chosenAutoPath.getName().contains("CHARGE")) {
      swerve.setWheelsUp();
      return;
    }
    // Auto is over, stop the claw
    // If we enabled in autonomous, this will stop stuff 
    // to give you an indicator of timing
    if (timer.get() > 15) {
      claw.setDesiredSpeed(PlacementConstants.CLAW_STOPPED_SPEED);
      return;
    }
    // Keep the autonomous states updated
    autoSegmentedWaypoints.periodic();
    // If we are close to the grid, allow the camera to modify odometry
    // This is because the camera was getting inacurate at longer distances
    if ((DriverStation.getAlliance() == Alliance.Red && swerve.getPose().getX() > 13) ||
        DriverStation.getAlliance() == Alliance.Blue && swerve.getPose().getX() < 3.5)// || autoSegmentedWaypoints.halfway) 
        // commenting autoalign.halfway becuase we dont want the camera to interfere while we are way out far
    {
      autoAlignment.calibrateOdometry();
    }
  }

  @Override
  public void teleopInit() {
    // Reset the timer for teleopinit 
    timer.restart();
    autoAlignment.setConeMode(true);
    arduinoController.setLEDState(LEDConstants.ARM_YELLOW);
    DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
    arm.setBrakeMode();
    DriverUI.enabled = true;
    DriverUI.freshCode = false;
    SwerveTrajectory.resetTrajectoryStatus();
    autoAlignment.setConeOffset(0);
  }

  @Override
  public void teleopPeriodic() {

    swerve.periodic();
    arm.periodic();
    claw.periodic();
    autoAlignment.calibrateOdometry();

    // If we are in the last 100 ms of the match, set the wheels up
    // This is to prevent any charge pad sliding
    if (timer.get() > 134.9) {
      swerve.setWheelsUp();
      claw.setDesiredSpeed(PlacementConstants.CLAW_OUTTAKE_SPEED_CONE);
      return;
    }

    // Find the max angular velocity of the drivetrain using the arm location
    DriveConstants.DYNAMIC_MAX_ANGULAR_SPEED = armCalculations.lerp(
      DriveConstants.MIN_ANGULAR_SPEED, 
      DriveConstants.MAX_ANGULAR_SPEED,
      MathUtil.clamp(1-(arm.getXPosition()/40.0), 0, 1)
    );

    // Get the driver's inputs and apply deadband; Note that the Y axis is inverted
    // This is to ensure that the up direction on the joystick is positive inputs
    double driverLeftX = MathUtil.applyDeadband(-driver.getLeftX(), OIConstants.DRIVER_DEADBAND);
    double driverLeftY = MathUtil.applyDeadband(-driver.getLeftY(), OIConstants.DRIVER_DEADBAND);
    double driverRightX = MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADBAND);
    double driverRightY = MathUtil.applyDeadband(driver.getRightY(), OIConstants.DRIVER_DEADBAND);

    double operatorLeftX = MathUtil.applyDeadband(operator.getLeftX(), OIConstants.OPERATOR_DEADBAND);
    double operatorLeftY = MathUtil.applyDeadband(operator.getLeftY(), OIConstants.OPERATOR_DEADBAND);
    double operatorRightX = MathUtil.applyDeadband(operator.getRightX(), OIConstants.OPERATOR_DEADBAND);
    double operatorRightY = MathUtil.applyDeadband(operator.getRightY(), OIConstants.OPERATOR_DEADBAND);

    Translation2d operatorLeftAxis = OICalc.toCircle(operatorLeftX, operatorLeftY);

    Translation2d driverLeftAxis = OICalc.toCircle(driverLeftX, driverLeftY);

    // If we are on blue alliance, flip the driverLeftAxis
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      driverLeftAxis = driverLeftAxis.unaryMinus();
    }

    if (SwerveTrajectory.trajectoryStatus.equals("setup") && !driver.getAButton()) {
      autoAlignment.setNearestValues();
    }

    // If the driver hits the start/back button, reset the ROTATION of the bot
    // to assume that it is directly facing the driver.
    // This is used in dire situations where we don't want to go as far as to find a tag to align to, 
    // but something happened at some point which heavily messed up our alignment with the field
    if (driver.getStartButtonPressed() || driver.getBackButtonPressed()) {
        // When the robot is facing the red alliance driver station, 
        // it is considered to be at 0 degrees
        swerve.resetOdometry(
            new Pose2d(
                swerve.getPose().getTranslation(), 
                Rotation2d.fromDegrees(
                    DriverStation.getAlliance() == DriverStation.Alliance.Red 
                    ? 0 
                    : 180))
        );
    }

    // When not aligning, reset the max speed to the teleop speed
    if (driver.getAButtonReleased()) {
      DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
      SwerveTrajectory.resetTrajectoryStatus();
      SwerveTrajectory.HDC.getThetaController().reset(swerve.getYaw().getRadians());
      //autoAlignment.setConeOffset(AutoAlignment.coneMode ? 1 : 0);
    }
    else if (driver.getAButton()) {

      if (driver.getAButtonPressed()) {
        // Slow the drive down for consistency
        DriveConstants.MAX_SPEED_METERS_PER_SECOND = AlignmentConstants.MAX_SPEED_METERS_PER_SECOND;
        // This resets the "momentum" of the path
        SwerveTrajectory.resetTrajectoryStatus();
        // This resets the "rotational momentum" of the integerals
        SwerveTrajectory.HDC.getThetaController().reset(swerve.getYaw().getRadians());
        // Reset the autoAlignment timer
        // This is so that the robot can change its "closest" node 
        // in the first second of aligning
        autoAlignment.resetTimer();
        
      }
      
      autoAlignment.alignToTag(driverLeftAxis.getY());
      
      // Since we reset the timer when AButtonPressed(), for the first one second of alignment, 
      // correct for any error in the odometry by resetting the nearest alignment offset. 
      // This will work since the robot will be rotating, 
      // and the camera will be writing the actual position of the robot.
      if (autoAlignment.getAlignmentTimer() < 1) {
        autoAlignment.setNearestAlignmentOffset();
      }

    } else if (driver.getRightBumper()) {

      if (driver.getRightBumperPressed()) {

        autoAlignment.startChargePad();

      }

      autoAlignment.chargeAlign();

    // Use the left bumper as a sort of e-stop for the swerve
    } else if (driver.getLeftBumper()) {

      swerve.setWheelsX();

    } else {
      // If the driver holds the left stick button, the robot will snap to the nearest 180 degree angle
      if (driver.getRightStickButton()) {
        // Reset the integrals at the start of clicking the auto-rotation button
        if (driver.getRightStickButtonPressed()) {
          SwerveTrajectory.HDC.getThetaController().reset(swerve.getYaw().getRadians());
        }
        autoAlignment.snapToAngle(driverLeftAxis, Rotation2d.fromDegrees(Math.abs(swerve.getYaw().getDegrees()) > 90 ? 180 : 0));
      }
      // If the driver holds the Y button, the robot will drive relative to itself
      // This is useful for driving in a straight line (backwards to intake!)
      else if (driver.getYButton()) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          swerve.drive(-driverLeftAxis.getY(), -driverLeftAxis.getX(), -driverRightX * 0.25, false, false);
        }
        else {
          swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), -driverRightX * 0.25, false, false);
        }
      }
      else {
        // Flip the X and Y inputs to the swerve drive because going forward (up) is positive Y on a controller joystick
        swerve.drive(driverLeftAxis.getY(), driverLeftAxis.getX(), -driverRightX * 0.25, true, true);
      }
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
      
      arm.drive(new Translation2d(operatorLeftAxis.getX(), -operatorLeftAxis.getY()));
      
    }

    // The moment the robot takes in a cone/cube,
    // The operator can set the robot into the desired mode
    if (operator.getXButton()) {
      autoAlignment.setConeMode(false);
      arduinoController.setLEDState(LEDConstants.ARM_PURPLE);
    }
    else if (operator.getYButton()) {
      autoAlignment.setConeMode(true);
      arduinoController.setLEDState(LEDConstants.ARM_YELLOW);
    }

    // POV = D-Pad...
    switch (OICalc.getDriverPOVPressed(driver.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        break;

      // Clicking down
      case 180:
        break;

      // Clicking left
      case 270:
        // If we are focusing on a substation, change the substation offset multiplier, not the cone offset multiplier.
        if (autoAlignment.getTagID() == 4 || autoAlignment.getTagID() == 5) {
          autoAlignment.setSubstationOffset((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1);
        }
        else {
          autoAlignment.setConeOffset(autoAlignment.getConeOffset() + ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        }
        break;

      // Clicking right
      case 90:
        // If we are focusing on a substation, change the substation offset multiplier, not the cone offset multiplier.
        if (autoAlignment.getTagID() == 4 || autoAlignment.getTagID() == 5) {
          autoAlignment.setSubstationOffset((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1);
        }
        else {
          autoAlignment.setConeOffset(autoAlignment.getConeOffset() - ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 1 : -1));
        }
        break;
    }

    // POV = D-Pad...
    switch (OICalc.getOperatorPOVPressed(operator.getPOV())) {
      // Not clicked
      case -1:
        break;

      // Clicking up
      case 0:
        boolean hotReloadHigh = arm.getArmIndex() == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX;
        arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_HIGH_PREP_INDEX : PlacementConstants.CUBE_HIGH_INDEX);
        if (!hotReloadHigh) { arm.startTrajectory((AutoAlignment.coneMode) ? PlacementConstants.HIGH_CONE_TRAJECTORY : PlacementConstants.HIGH_CUBE_TRAJECTORY); }
        break;

      // Clicking down
      case 180:
        arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_INTAKE_INDEX : PlacementConstants.CUBE_INTAKE_INDEX);
        break;

      // Clicking left
      case 270:
        /*
         * If we are on blue alliance, make it so that the left and right buttons set the arm to human tag
         *   when the robot is halfway across the field
         * this code is in place due to our operator confusing the buttons and clicking the wrong one.
         */
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                if (swerve.getPose().getTranslation().getX() > AlignmentConstants.FIELD_WIDTH_METERS/2) {
                    arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
                }
                else {
                    boolean hotReloadMid = arm.getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX;
                    arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_MID_PREP_INDEX : PlacementConstants.CUBE_MID_INDEX);
                    if (!hotReloadMid && AutoAlignment.coneMode) { arm.startTrajectory(PlacementConstants.MID_CONE_TRAJECTORY); }
                }
        }
        // we are on red alliance,
        else {
            if (swerve.getPose().getTranslation().getX() < AlignmentConstants.FIELD_WIDTH_METERS/2) {
                arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
            }
            else {
                boolean hotReloadMid = arm.getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX;
                arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_MID_PREP_INDEX : PlacementConstants.CUBE_MID_INDEX);
                if (!hotReloadMid && AutoAlignment.coneMode) { arm.startTrajectory(PlacementConstants.MID_CONE_TRAJECTORY); }
            }
        }
        break;

      // Clicking right
      case 90:
        /*
         * If we are on blue alliance, make it so that the left and right buttons set the arm to human tag
         *   when the robot is halfway across the field
         * this code is in place due to our operator confusing the buttons and clicking the wrong one.
         */
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                if (swerve.getPose().getTranslation().getX() > AlignmentConstants.FIELD_WIDTH_METERS/2) {
                    arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
                }
                else {
                    boolean hotReloadMid = arm.getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX;
                    arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_MID_PREP_INDEX : PlacementConstants.CUBE_MID_INDEX);
                    if (!hotReloadMid && AutoAlignment.coneMode) { arm.startTrajectory(PlacementConstants.MID_CONE_TRAJECTORY); }
                }
        }
        // we are on red alliance,
        else {
            if (swerve.getPose().getTranslation().getX() < AlignmentConstants.FIELD_WIDTH_METERS/2) {
                arm.setArmIndex(PlacementConstants.HUMAN_TAG_PICKUP_INDEX);
            }
            else {
                boolean hotReloadMid = arm.getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX;
                arm.setArmIndex((AutoAlignment.coneMode) ? PlacementConstants.CONE_MID_PREP_INDEX : PlacementConstants.CUBE_MID_INDEX);
                if (!hotReloadMid && AutoAlignment.coneMode) { arm.startTrajectory(PlacementConstants.MID_CONE_TRAJECTORY); }
            }
        }
        break;
    }
    if (operator.getBButtonPressed()){
      arm.setArmIndex(PlacementConstants.CONE_FLIP_INDEX);
    }
    if (operator.getRightStickButtonPressed()) {
      if (arm.getAtPlacementPosition()) {
        claw.outTakeforXSeconds(AutoAlignment.coneMode ? 0.1 : 0.3);
        // Attempt at making a small animation of the leds showcasing the event
        // arduinoController.setLEDState(AutoAlignment.coneMode ? LEDConstants.BELLY_PAN_YELLOW : LEDConstants.BELLY_PAN_RED, true);
      }
      else {
        arm.setArmIndex(PlacementConstants.STOWED_INDEX);
      }
    }
    if (operator.getAButton()) {
      arm.finishPlacement();
    }

    // These buttons are right below the Xbox logo, aimed to be a bit out of reach
    // (after all we don't want to accidentally press them)
    if (operator.getBackButtonPressed()) {
      arm.setArmIndex(PlacementConstants.ARM_FLIP_INDEX);
    }
    else if (operator.getStartButtonPressed()) {
      arm.setArmIndex(PlacementConstants.ARM_FLIP_INDEX);
    }

    /* Claw speed controls:
     *   If the left bumper is pressed, e-stop the claw
     *   If the right bumper is held, or if teleop just started,
     *   outtake an object when the arm is at placement position
     *   If the left trigger is held, intake an object:
     *     Keep the fastest intake speed until the claw is e-stopped/reversed
     *     This is to allow the trigger to be fully pressed intake an object,
     *   and then let go to keep the claw at the same speed
     *   If the right trigger is held, manually outtake an object (try to use the right bumper instead)
     */
    if (operator.getLeftBumper()) {

      claw.stopClaw();

    } else if ((operator.getRightBumper()) && !claw.getStartedOuttakingBool() || (timer.get() < 0.1 && !DriverStation.isTestEnabled())) {
      // Check if the arm has completed the path to place an object
      if (arm.getAtPlacementPosition()) {
        claw.outTakeforXSeconds(AutoAlignment.coneMode ? 0.1 : 0.3);
        // Attempt at making a small animation of the leds showcasing the event
        // arduinoController.setLEDState(AutoAlignment.coneMode ? LEDConstants.BELLY_PAN_YELLOW : LEDConstants.BELLY_PAN_RED, true);
      }

    } else if (operator.getLeftTriggerAxis() > 0) {

      claw.setDesiredSpeed(operator.getLeftTriggerAxis());

    } else if (operator.getRightTriggerAxis() > 0) {

      claw.setDesiredSpeed(-operator.getRightTriggerAxis());

    } else if (claw.getFinishedOuttaking() && arm.getAtPlacementPosition()) {

      if (arm.getArmIndex() == PlacementConstants.CONE_HIGH_PLACEMENT_INDEX ||
            arm.getArmIndex() == PlacementConstants.CONE_HIGH_PREP_TO_PLACE_INDEX) {
      
        arm.setArmIndex(PlacementConstants.HIGH_TO_STOWED_INDEX);
        arm.startTrajectory(PlacementConstants.HIGH_TO_STOWED_TRAJECTORY);

        } else if (arm.getArmIndex() == PlacementConstants.CONE_MID_PREP_TO_PLACE_INDEX ||
                arm.getArmIndex() == PlacementConstants.CONE_MID_PLACEMENT_INDEX) 
        {

            arm.setArmIndex(PlacementConstants.MID_TO_STOWED_INDEX);

        } else {

            arm.setArmIndex(PlacementConstants.STOWED_INDEX);
        
        }

        claw.setStartedOuttakingBool(false);
        claw.setFinishedOuttaking(false);

        claw.stopClaw();

    }

    // Blink the lights red if we are flipped over
    if (Math.abs(swerve.getPitch().getDegrees()) > 35) {

      arduinoController.setLEDState(LEDConstants.BELLY_PAN_FLASH_RED);
    
    }
    // Controller rumble settings:
    // If the robot is close to the desired grid index, rumble both controllers
    // This is to help the driver know when to stop moving the robot if manually aligning
    // Or as a confirmation for auto alignment
    else if (autoAlignment.getCurrentNorm() < (PlacementConstants.CONE_BASE_DIAMETER/2) && (autoAlignment.getCurrentNorm() != -1)) {
    
      driver.setRumble(RumbleType.kBothRumble, 1);
      operator.setRumble(RumbleType.kBothRumble, 1);

      arduinoController.setLEDState(LEDConstants.BELLY_PAN_GREEN);

      DriverUI.aligned = true;
    
    }
    else {
    
      if (autoAlignment.getCurrentNorm() < 1 && (autoAlignment.getCurrentNorm() != -1)) {
      
        arduinoController.setLEDState(LEDConstants.BELLY_PAN_RED);

        DriverUI.aligned = false;
    
      }
      else {
    
        arduinoController.setLEDState(AutoAlignment.coneMode ? LEDConstants.BELLY_PAN_YELLOW : LEDConstants.BELLY_PAN_PURPLE);
      
      }
    
      driver.setRumble(RumbleType.kBothRumble, 0);
      operator.setRumble(RumbleType.kBothRumble, 0);
    
    }

    //Rumble the claw if it is stalling, judged by whether the claw is drawing more amps than a preset limit.
    if (claw.getHasGameElement()) {
    
        driver.setRumble(RumbleType.kBothRumble, 0.25);
        operator.setRumble(RumbleType.kBothRumble, 0.25);
        
        if (claw.justAquiredGameElement()) {
            arduinoController.setLEDState(
                AutoAlignment.coneMode 
                    ? LEDConstants.BELLY_PAN_YELLOW_BLINK 
                    : LEDConstants.BELLY_PAN_PURPLE_BLINK, 
                true);
        }
    
    }
    
    if (arm.halfwayFinishedWithConeFlip()) {
       claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED_CONE);
    }
  }

  @Override
  public void testInit() {
    DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
    arm.setBrakeMode();
    DriverUI.enabled = true;
    DriverUI.freshCode = false;
    SwerveTrajectory.resetTrajectoryStatus();
    autoAlignment.setConeOffset(0);
    // Stop the timer, since this is test mode
    // we want to allow the robot to be enabled as much
    // as we want.
    timer.reset();
    timer.stop();
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }
}
