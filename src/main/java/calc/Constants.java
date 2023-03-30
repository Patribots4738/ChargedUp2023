// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package calc;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static double MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
        public static double DYNAMIC_MAX_ANGULAR_SPEED = 3 * Math.PI; // radians per second
        public static final double MIN_ANGULAR_SPEED = 4 * Math.PI; // radians per second
        // NEEDS TO BE TESTED vvv
        public static final double MAX_ANGULAR_SPEED = 8 * Math.PI; // radians per second

        public static final double MAX_TELEOP_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.51);

        public static final double DIRECTION_SLEW_RATE = 6.28; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 80.0; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 80.0; // percent per second (1 = 100%)

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                           // Front Positive,   Left Positive
            new Translation2d( WHEEL_BASE / 2,  TRACK_WIDTH / 2),  // Front Left
            new Translation2d( WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // Front Right
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2),  // Rear Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Rear Right

        // Angular offsets of the modules relative to the chassis in radians
        // add 90 degrees to change the X and Y axis
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180+90);
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90+90);
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90+90);
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0+90);

        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        public static final boolean GYRO_REVERSED = true;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 12;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_P = 1;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class AutoConstants {

        public static final double MAX_SPEED_METERS_PER_SECOND = 4; // previously 1.75
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.25; // 2.5; (2.5vel and 2.5accel output 3s runtime on _1_A)
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 3 * Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI/1.7; 

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        public static final double X_CORRECTION_P = 2.5;//7;
        public static final double X_CORRECTION_I = 0;
        public static final double X_CORRECTION_D = 0.2;

        public static final double Y_CORRECTION_P = 2.5;//6.03;
        public static final double Y_CORRECTION_I = 0;
        public static final double Y_CORRECTION_D = 0.2;

        public static final double ROTATION_CORRECTION_P = .63;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0;

        // Constraint for the motion-profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class ClawConstants {
        public static final int CLAW_CAN_ID = 12;

        public static final double CLAW_POSITION_CONVERSION_FACTOR = 25;

        public static final double CLAW_P = 1;
        public static final double CLAW_I = 0;
        public static final double CLAW_D = 0;
        public static final double CLAW_FF = 1;

        public static final double CLAW_MIN_OUTPUT = -1;
        public static final double CLAW_MAX_OUTPUT = 1;

        public static final int CLAW_STALL_LIMIT = 25;
        public static final int CLAW_FREE_LIMIT = 30;


        // from the edge of the upper arm, straight out,
        // and stop at the end of the 3d-printed "grabber"
        public static final double CLAW_LENGTH_INCHES = 13;
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static final class ArmConstants {

      public static final int LOWER_ARM_LEFT_MOTOR_CAN_ID = 10;
      public static final int LOWER_ARM_RIGHT_MOTOR_CAN_ID = 9;
      public static final int UPPER_ARM_MOTOR_CAN_ID = 11;

      // The gear ratio of the lower arm is 84:1
      public static final double LOWER_ARM_GEAR_RATIO = (84);

      // The gear ratio of the upper arm is 115:1
      public static final double UPPER_ARM_GEAR_RATIO = (115);

      // UNFINISHED, CURRENT LIMITS TO SLOW MOTORS
      public static final int LOWER_FREE_LIMIT = 50;
      public static final int LOWER_STALL_LIMIT = 50;
      // Make the max RPM equal to 1/4 rotations per second
      public static final int LOWER_MAX_RPM = ((int) (LOWER_ARM_GEAR_RATIO) * 15);

      public static final int UPPER_FREE_LIMIT = 50;
      public static final int UPPER_STALL_LIMIT = 50;
      // Make the max RPM equal to 1/4 rotations per second

      public static final int UPPER_MAX_RPM = ((int) (UPPER_ARM_GEAR_RATIO) * 15);

      // The length of the first pivot point to the second pivot point, in inches
      public static final double LOWER_ARM_LENGTH = 34.096;

      // The length of the second pivot point to the claw, in inches
      public static final double UPPER_ARM_LENGTH = 23;

      // The max reach of the bot horizontally, in inches
      // This is not taking into account the claw length, and is only for IK
      public static final double MAX_REACH = LOWER_ARM_LENGTH + UPPER_ARM_LENGTH;

      public static final double MIN_REACH = LOWER_ARM_LENGTH - UPPER_ARM_LENGTH;

      // Limit the max Y reach of the arm, due to the rules stating we cannot be over 6'6"
      // 78 inches is the rule, 11 inches is the base of the pivot of the arm to the ground
      public static final double MAX_REACH_Y = 78 - 11 - ClawConstants.CLAW_LENGTH_INCHES;
      public static final double MAX_REACH_X = 48 + (Units.metersToInches(PlacementConstants.ROBOT_LENGTH_METERS) / 2.0) - ClawConstants.CLAW_LENGTH_INCHES;

      // When the arm is near the top of the limit, the arm will flip to the other solution
      // This is to prevent the upper arm from going above the limit when that happens.
      public static final double ARM_FLIP_X = 6.5;

      // Multiply the absolute encoder output to get radians instead of rotations
      public static final double LOWER_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
      public static final double UPPER_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians

      public static final double LOWER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60; // Radians per second
      public static final double UPPER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60; // Radians per second

      // The number of degrees that the upper arm can rotate from parallel to the front of the bot
      // Add 90 to lower arm's upper limit b/c the zero is perpendicular to chassis
      public static final double LOWER_ARM_LOWER_LIMIT = 1.6;
      public static final double LOWER_ARM_UPPER_LIMIT = 4.3;

      // The number of degrees that the upper arm can rotate from the base of the lower arm
      // Add 23 to the lower limit because the upper arm can only start 23* from the lower arm
      public static final double UPPER_ARM_LOWER_LIMIT = 0.34;
      public static final double UPPER_ARM_UPPER_LIMIT = 5.7;

      // The amount of error allowed for the arm's position, in Radians
      // This is primarily used in autonomous
      public static final double LOWER_ARM_DEADBAND_COARSE = Math.toRadians(15);
      public static final double UPPER_ARM_DEADBAND_COARSE = Math.toRadians(20);

      public static final double LOWER_ARM_DEADBAND_FINE = Math.toRadians(7);
      public static final double UPPER_ARM_DEADBAND_FINE = Math.toRadians(15);


      // The outputs used in the output range for the lower and upper arms
      public static final double LOWER_MAX_OUTPUT = 0.85;
      public static final double LOWER_MIN_OUTPUT = -LOWER_MAX_OUTPUT;

      public static final double UPPER_MAX_OUTPUT = 0.85;
      public static final double UPPER_MIN_OUTPUT = -UPPER_MAX_OUTPUT;

      /**
       * SysID values:
       * <p>
       * Lower:
       * P = 0.0032326
       * D = 0.0012612
       * <p>
       */
      // PID values for the lower and upper arm
      public static final double LOWER_P = 0.8;//0.25;
      public static final double LOWER_I = 0.0001;
      public static final double LOWER_D = 0.1;
      public static final double LOWER_FF = 1;

      public static final double LOWER_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
      public static final double LOWER_ENCODER_POSITION_PID_MAX_INPUT = LOWER_ENCODER_POSITION_FACTOR; // radians

      public static final double UPPER_P = 0.8;//.25;
      public static final double UPPER_I = 0.0001;
      public static final double UPPER_D = 0.3;
      public static final double UPPER_FF = 1;

      public static final double UPPER_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
      public static final double UPPER_ENCODER_POSITION_PID_MAX_INPUT = UPPER_ENCODER_POSITION_FACTOR; // radians

      /**
       * FF is a predictive formula that uses the input of where we want to be to predict the path required to get there
       * This is an open loop, and thus unable to react to the effects of disturbances / unknown disturbances.
       * Because of this, we need to give it disturbance data, which is why we use sysID
       * It then uses the data to predict the output of the motor, creating a more accurate prediction
       * <p>
       * In contrast, a closed loop is commonly used as PID, looking and reacting to the current position
       * in opposition to the desired position, taking note of any disturbance that affected the motor.
       * Since weight, and more importantly, the things connected to the arm are changing,
       * making it difficult to perfectly tune
       * <p>
       * We are using a very small combination of both, using the FF to predict the path, and PID to correct for error
       * FF will be doing the heavy lifting, and PID will only be small finishing touches for perfection.
       * 👌
       */
      public static final double S_LOWER = -0.10452;
      public static final double G_LOWER = 0;
      public static final double V_LOWER = 10.914;
      public static final double A_LOWER = 1.7823;

      public static final double S_UPPER = 0.1382;
      public static final double G_UPPER = 0; //1.0051; // (We don't like gravity)
      public static final double V_UPPER = 1.7457;
      public static final double A_UPPER = 0.6718;
    }

    public static final class LEDConstants {

      public static final int ARDUINO_ADDRESS = 8;
      public static final int BELLY_PAN_RAINBOW = 0;
      // The "theater chase" could possibly induce seizures,
      // Please refrain from using these.
      @Deprecated
      public static final int BELLY_PAN_THEATER_CHASE_RAINBOW = 1;
      public static final int BELLY_PAN_RED_ALLIANCE = 2;
      public static final int BELLY_PAN_FLASH_RED = 3;
      public static final int BELLY_PAN_RED = 4;
      public static final int BELLY_PAN_BLUE = 5;
      public static final int BELLY_PAN_GREEN = 6;
      public static final int BELLY_PAN_PURPLE = 7;
      public static final int BELLY_PAN_YELLOW = 8;
      public static final int BELLY_PAN_BLACK = 9;


      public static final int SPONSOR_PANEL_RAINBOW = 10;
      public static final int SPONSOR_PANEL_THEATER_CHASE_RAINBOW = 11;
      public static final int SPONSOR_PANEL_THEATER_CHASE = 12;
      public static final int SPONSOR_PANEL_FLASH_RED = 13;
      public static final int SPONSOR_PANEL_RED = 14;
      public static final int SPONSOR_PANEL_BLUE = 15;
      public static final int SPONSOR_PANEL_GREEN = 16;
      public static final int SPONSOR_PANEL_PURPLE = 17;
      public static final int SPONSOR_PANEL_YELLOW = 18;
      public static final int SPONSOR_PANEL_BLACK = 19;


      public static final int ARM_RAINBOW = 20;
      public static final int ARM_THEATER_CHASE_RAINBOW = 21;
      public static final int ARM_THEATER_CHASE = 22;
      public static final int PROGRAM_23 = 23;
      public static final int ARM_RED = 24;
      public static final int ARM_BLUE = 25;
      public static final int ARM_GREEN = 26;
      public static final int ARM_PURPLE = 27;
      public static final int ARM_YELLOW = 28;
      public static final int ARM_BLACK = 29;


      public static final int CLAW_RAINBOW = 30;
      public static final int CLAW_THEATER_CHASE_RAINBOW = 31;
      public static final int CLAW_THEATER_CHASE = 32;
      public static final int PROGRAM_33 = 33;
      public static final int CLAW_RED = 34;
      public static final int CLAW_BLUE = 35;
      public static final int CLAW_GREEN = 36;
      public static final int CLAW_PURPLE = 37;
      public static final int CLAW_YELLOW = 38;
      public static final int CLAW_BLACK = 39;
    }

    public static final class VisionConstants {

      public static final String CAMERA_1_NAME = "FrontCam";
      public static final String CAMERA_2_NAME = "BackCam";
      public static final double AMBIGUITY_THRESHOLD = 0.06;

      // Distance from the camera to the center
      public static final Transform3d CAMERA_1_POSITION = new Transform3d(
          new Translation3d(
              // Forward
              Units.inchesToMeters(2),
              // Left (neg because it's right)
              Units.inchesToMeters(-6),
              // Up
              Units.inchesToMeters(10.25)),
          new Rotation3d(
              0,
              0,
              0));

      // Distance from the camera to the center
      public static final Transform3d CAMERA_2_POSITION = new Transform3d(
          new Translation3d(
              // Forward (neg because it's backwards)
              Units.inchesToMeters(-10),
              // Left 
              Units.inchesToMeters(-6.125),
              // Up
              Units.inchesToMeters(9.438)),
          new Rotation3d(
              0,
              180,
              0));
    }

    public static final class AlignmentConstants {


        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double SNAP_TO_ANGLE_P = 0.0025;
        

        public static final double CONE_OFFSET_METERS = 0.542615;
        private static final double GRID_TAG_HEIGHT_METERS = Units.inchesToMeters(18.22);
        private static final double HUMAN_TAG_HEIGHT_METERS = Units.inchesToMeters(27.38);
        public static final double GRID_BARRIER_METERS = Units.inchesToMeters(12); // real is 14-15
        public static final double SUBSTATION_OFFSET_METERS = 0.7;
        public static final double ALLOWABLE_ERROR_METERS = Units.inchesToMeters(2);
        public static final double FIELD_WIDTH_METERS = 16.53;

        public static final double CHARGE_PAD_CORRECTION_P = 0.05;

        // The below code is not used, and is only here for reference...
        // We instead use PhotonCameraPose.AprilTagFieldLayout
        // private final Pose3d TAG_0_POSE = null;
        // private final Pose3d TAG_1_POSE = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), GRID_TAG_HEIGHT_METERS, new Rotation3d(0, 0, 0));
        // private final Pose3d TAG_2_POSE = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.91), GRID_TAG_HEIGHT_METERS, new Rotation3d(0, 0, 0));
        // private final Pose3d TAG_3_POSE = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), GRID_TAG_HEIGHT_METERS, new Rotation3d(0, 0, 0));
        // private final Pose3d TAG_4_POSE = new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), HUMAN_TAG_HEIGHT_METERS, new Rotation3d(0, 0, 0));
        // private final Pose3d TAG_5_POSE = new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), HUMAN_TAG_HEIGHT_METERS, new Rotation3d(0, 0, Units.degreesToRadians(180)));
        // private final Pose3d TAG_6_POSE = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), GRID_TAG_HEIGHT_METERS, new Rotation3d(0, 0, Units.degreesToRadians(180)));
        // private final Pose3d TAG_7_POSE = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), GRID_TAG_HEIGHT_METERS, new Rotation3d(0, 0, Units.degreesToRadians(180)));
        // private final Pose3d TAG_8_POSE = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), GRID_TAG_HEIGHT_METERS, new Rotation3d(0, 0, Units.degreesToRadians(180)));
    }

    public static final class PlacementConstants {

      // The length of the robot
      public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(25);
      // Length of the bumpers on the robot
      public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(2.75);

      public static final double CONE_BASE_DIAMETER = Units.inchesToMeters(6.629);

      public static final int STOWED_INDEX = 0;
      public static final int HYBRID_PLACEMENT_INDEX = 1;
      public static final int MID_CONE_PLACEMENT_INDEX = 2;
      public static final int HIGH_CONE_PLACEMENT_INDEX = 3;
      public static final int HUMAN_TAG_PICKUP_INDEX = 4;
      public static final int MID_CUBE_LAUNCH_INDEX = 5;
      public static final int HIGH_CUBE_LAUNCH_INDEX = 6;
      public static final int FLOOR_INTAKE_INDEX = 7;
      public static final int SOLUTION_FLIP_INDEX_POSITIVE = 8;
      public static final int SOLUTION_FLIP_INDEX_NEGATIVE = 9;
      public static final int HIGH_TO_STOWWED_INDEX = 10;
      public static final int LONG_ARM_REACH_INDEX = 11;
      public static final int FLOOR_INTAKE_PREP_INDEX = 12;
      public static final int HIGH_CONE_PREP_INDEX = 13;
      public static final int HIGH_CONE_PREP_TO_PLACE_INDEX = 14;
      public static final int CONE_INTAKE_INDEX = 15;
      public static final int CUBE_INTAKE_INDEX = 16;
      public static final int CONE_FLIP_INDEX = 17;
      public static final int MID_CONE_PREP_INDEX = 18;
      public static final int MID_CONE_PREP_TO_PLACE_INDEX = 19;
      public static final int AUTO_INIT_INDEX = 20;
      public static final int AUTO_CUBE_INTAKE_INDEX = 21;

      public static final double CLAW_INTAKE_SPEED_CONE = 1;
      public static final double CLAW_OUTTAKE_SPEED_CONE = -1;
      public static final double CLAW_INTAKE_SPEED_CUBE = 0.8;
      public static final double CLAW_OUTTAKE_SPEED_CUBE = -0.3;

      public static final double CLAW_STOPPED_SPEED = 0;
      
      public static final Translation2d SOLUTION_FLIP_TRANSITION_POINT_START = new Translation2d(0,35);
      public static final Translation2d SOLUTION_FLIP_TRANSITION_POINT_FINISH = new Translation2d(0,30);
      public static final Translation2d SOLUTION_FLIP_POSIITON_POSITIVE = new Translation2d(28, 45);
      public static final Translation2d SOLUTION_FLIP_POSIITON_NEGATIVE = new Translation2d(-28,45);

      // public static final Translation2d FLOOR_INTAKE_PREP_POSITION = new Translation2d(19, 10);
      // public static final Translation2d FLOOR_INTAKE_POSITION = new Translation2d(19, 2);

      public static final Translation2d CUBE_INTAKE_POSITION_PREP = new Translation2d((((Units.metersToInches(ROBOT_LENGTH_METERS)/2.0) + Units.metersToInches(BUMPER_LENGTH_METERS))), 15);
      public static final Translation2d CONE_INTAKE_POSITION_PREP = new Translation2d(30, 5);
      public static final Translation2d CUBE_INTAKE_POSITION = new Translation2d(18, 6);
      public static final Translation2d CUBE_INTAKE_POSITION_AUTO = new Translation2d(40, 8);
      public static final Translation2d BUMPER_INTAKE_POSITION = new Translation2d((((Units.metersToInches(ROBOT_LENGTH_METERS)/2.0) + Units.metersToInches(BUMPER_LENGTH_METERS))), 5);

      
      public static final Translation2d CONE_FLIP_POSITION_0 = new Translation2d(12.6, 8);
      // public static final Translation2d CONE_FLIP_POSITION_0 = new Translation2d(25, 5);
      
      public static final Translation2d TRANSITION_POSITION = new Translation2d(10, 37);
      public static final Translation2d STOWED_POSITION = new Translation2d(3.5, 14);
      public static final Translation2d AUTO_INIT_PICKUP = new Translation2d(12.43, 8);

      public static final Translation2d HYBRID_POSITION = new Translation2d(14, 13);
      public static final Translation2d ARM_MEDIUM_GRID_POSITION = new Translation2d(28, 27);
      public static final Translation2d ARM_HIGH_GRID_POSITION = new Translation2d(43, 35);
        
      public static final Translation2d MID_CONE_POSITION_0 = new Translation2d(29, 32.88);
      public static final Translation2d MID_CONE_POSITION_1 = new Translation2d(30.5, 23.00);
      public static final Translation2d HIGH_CONE_POSITION_0 = new Translation2d(29.03, 46.46);
      public static final Translation2d HIGH_CONE_POSITION_1 = new Translation2d(46.35, 33);
      public static final Translation2d HIGH_CONE_POSITION_2 = new Translation2d(48, 26);
      public static final Translation2d HUMAN_TAG_PICKUP = new Translation2d(15, 32);
      public static final Translation2d CUBE_HIGH_LAUNCH = new Translation2d(35, 38);
      public static final Translation2d CUBE_MID_LAUNCH = new Translation2d(20,25);

      public static final Translation2d LONG_ARM_REACH_0 = new Translation2d(-ArmConstants.MAX_REACH_X, 30);
      public static final Translation2d LONG_ARM_REACH_1 = new Translation2d(-ArmConstants.MAX_REACH_X, 15);
      public static final Translation2d LONG_ARM_REACH_2 = new Translation2d(-ArmConstants.MAX_REACH_X, 5);

      public static final Translation2d HIGH_CONE_PREP = new Translation2d(24,41);
      public static final Translation2d MID_CONE_PREP = new Translation2d(24,35);

      public static final Translation2d[][] ARM_POSITIONS = {
        // Index 0 | Stowed
        {
          TRANSITION_POSITION,
          STOWED_POSITION
        },
        // Index 1 | Hybrid
        {
          TRANSITION_POSITION,
          HYBRID_POSITION
        },
        // Index 2 | Mid Cone
        {
          TRANSITION_POSITION,
          MID_CONE_POSITION_0,
          MID_CONE_POSITION_1
        },
        // Index 3 | High Cone
        {
          TRANSITION_POSITION,
          HIGH_CONE_POSITION_0,
          // ARM_HIGH_CONE_POSITION_1,
          HIGH_CONE_POSITION_2
        },
        // Index 4 | Human Tag
        {
          HUMAN_TAG_PICKUP
        },
        // Index 5 | Cube Mid
        {
          TRANSITION_POSITION,
          CUBE_MID_LAUNCH
        },
        // Index 6 | Cube High
        {
          TRANSITION_POSITION,
          CUBE_HIGH_LAUNCH
        },
        // Index 7 | Floor Intake
        {
          CUBE_INTAKE_POSITION_PREP,
          CUBE_INTAKE_POSITION
        },
        // Index 8 | Solution Flip Positive
        {
          SOLUTION_FLIP_TRANSITION_POINT_START,
          SOLUTION_FLIP_POSIITON_POSITIVE,
          SOLUTION_FLIP_TRANSITION_POINT_FINISH
        },
        // Index 9 | Solution Flip Negative
        {
          SOLUTION_FLIP_TRANSITION_POINT_START,
          SOLUTION_FLIP_POSIITON_NEGATIVE,
          SOLUTION_FLIP_TRANSITION_POINT_FINISH
        },
        // Index 10 | High To Stowed
        {
          HIGH_CONE_PREP,
          TRANSITION_POSITION,
          STOWED_POSITION
        },
        // Index 11 | Long Arm Reach
        {
          LONG_ARM_REACH_0,
          LONG_ARM_REACH_1,
          LONG_ARM_REACH_2
        },
        // Index 12 | Floor Intake Prep
        {
          CUBE_INTAKE_POSITION_PREP
        },
        // Index 13 | High Cone Prep
        {
          HIGH_CONE_PREP
        },
        // Index 14 | High Cone Prep to Place
        {
          HIGH_CONE_POSITION_0,
          HIGH_CONE_POSITION_2
        },
        // Index 15 | Cone Intake
        {
          CONE_INTAKE_POSITION_PREP,
          BUMPER_INTAKE_POSITION
        },
        // Index 16 | Cube Intake
        {
          CUBE_INTAKE_POSITION_PREP,
          CUBE_INTAKE_POSITION
        },
        // Index 17 | Cone Flip
        {
          CONE_FLIP_POSITION_0,
          CONE_INTAKE_POSITION_PREP,
          BUMPER_INTAKE_POSITION
        },
        // Index 18 | Mid Cone Prep
        {
          MID_CONE_PREP
        },
        // Index 19 | Mid Cone Prep to Place
        {
          MID_CONE_POSITION_0,
          MID_CONE_POSITION_1
        },
        // Index 20 | Auto Init Pickup
        {
          AUTO_INIT_PICKUP
        },
        // Index 21 | Cube Intake Auto
        {
          CONE_INTAKE_POSITION_PREP,
          CUBE_INTAKE_POSITION_AUTO
        }
      };
  }
}
