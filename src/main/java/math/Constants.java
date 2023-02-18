// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package math;

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
        public static final double MAX_SPEED_METERS_PER_SECOND = 2;
        public static final double MAX_ANGULAR_SPEED = 6 * Math.PI; // radians per second

        // Chassis configuration
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0);
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90);
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90);
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180);

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

        public static final double DRIVER_DEADBAND = 0.1;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        public static final double X_CORRECTION_P = 1;
        public static final double X_CORRECTION_I = 0;
        public static final double X_CORRECTION_D = 0;

        public static final double Y_CORRECTION_P = 0.3;
        public static final double Y_CORRECTION_I = 0;
        public static final double Y_CORRECTION_D = 0;

        public static final double ROTATION_CORRECTION_P = -0.22;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0;//.74;

        // Constraint for the motion-profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class ClawConstants {
        public static final int CLAW_CAN_ID = 12;

        public static final double CLAW_POSITION_CONVERSION_FACTOR = 50;

        public static final double CLAW_P = 1;
        public static final double CLAW_I = 0;
        public static final double CLAW_D = 0;
        public static final double CLAW_FF = 1;

        public static final double CLAW_MIN_OUTPUT = -1;
        public static final double CLAW_MAX_OUTPUT = 1;

        public static final int CLAW_STALL_LIMIT = 20;
        public static final int CLAW_FREE_LIMIT = 20;


        // from the edge of the upper arm, straight out,
        // and stop at the end of the 3d-printed "grabber"
        public static final double CLAW_LENGTH_X_INCHES = 13;
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static final class ArmConstants {

        public static final int LOWER_ARM_LEFT_MOTOR_CAN_ID = 9;
        public static final int LOWER_ARM_RIGHT_MOTOR_CAN_ID = 10;
        public static final int UPPER_ARM_MOTOR_CAN_ID = 11;

        // UNFINISHED, CURRENT LIMITS TO SLOW MOTORS
        public static final int UPPER_CURRENT_LIMIT = 80;
        public static final int LOWER_CURRENT_LIMIT = 80;

        // The length of the first pivot point to the second pivot point, in inches
        public static final double LOWER_ARM_LENGTH = 32;

        // The length of the second pivot point to the claw, in inches
        public static final double UPPER_ARM_LENGTH = 21;

        // The max reach of the bot horizontally, in inches
        public static final double MAX_REACH = LOWER_ARM_LENGTH + UPPER_ARM_LENGTH;

        // Limit the max Y reach of the arm, due to the rules stating we cannot be over 6'6"
        // 78 inches is the rule, 11 inches is the base of the arm to the ground
        public static final double MAX_REACH_Y = 78 - 11 - ClawConstants.CLAW_LENGTH_X_INCHES;

        // The gear ratio of the lower arm is 60:1
        public static final double LOWER_ARM_GEAR_RATIO = 60;

        // The gear ratio of the upper arm is 25:7
        public static final double UPPER_ARM_GEAR_RATIO = (85.0 / 9.0);

        // The factor that the encoder is multiplied by to get the actual position
        public static final double LOWER_ENCODER_POSITION_FACTOR = (LOWER_ARM_LENGTH * Math.PI) / (LOWER_ARM_GEAR_RATIO);
        public static final double UPPER_ENCODER_POSITION_FACTOR = (UPPER_ARM_LENGTH * Math.PI) / (UPPER_ARM_GEAR_RATIO);

        // The number of degrees that the upper arm can rotate EITHER WAY, in degrees
        public static final double LOWER_ARM_FREEDOM_DEGREES = 60;

        // The number of degrees that the upper arm can rotate EITHER WAY, in degrees
        public static final double UPPER_ARM_FREEDOM_DEGREES = 100;

        // The amount of error allowed for the arm's position, in FULL ROTATIONS
        // This is primarily used in autonomous
        public static final double LOWER_ARM_DEADBAND = 0.01;
        public static final double UPPER_ARM_DEADBAND = 0.01;

        // The outputs used in the output range for the lower and upper arms
        public static final double LOWER_MIN_OUTPUT = -1;
        public static final double LOWER_MAX_OUTPUT = 1;

        public static final double UPPER_MIN_OUTPUT = -1;
        public static final double UPPER_MAX_OUTPUT = 1;

        /**
         * SysID values:
         * <p>
         * Lower:
         * P = 0.0032326
         * D = 0.0012612
         * <p>
         */
        // PID values for the lower and upper arm
        public static final double LOWER_P = 0.1;
        public static final double LOWER_I = 0;
        public static final double LOWER_D = 0.025;
        public static final double LOWER_FF = 1;


        public static final double UPPER_P = 1;    //46.6;
        public static final double UPPER_D = 0; //34.2;
        public static final double UPPER_I = 0;//.035;
        public static final double UPPER_FF = 1;


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
        public static final double S_LOWER = 0;//.15999;
        public static final double G_LOWER = 0;//0.33966; (We don't like gravity)
        public static final double V_LOWER = 0;//.24941;
        public static final double A_LOWER = 0;//4.3455;

        public static final double S_UPPER = 0.47727;
        public static final double G_UPPER = 0; //1.0051; // (We don't like gravity)
        public static final double V_UPPER = 0.95764;
        public static final double A_UPPER = 0.19957;
    }

    public static final class VisionConstants {

        public static final String CAMERA_NAME = "Patribots4738";

        // Distance from the camera to the center
        public static final Transform3d CAMERA_POSITION = new Transform3d(
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

        public static final double CONE_OFFSET_METERS = 0.5;
    }

    public static final class AlignmentConstants {

        private static final double GRID_TAG_HEIGHT = Units.inchesToMeters(18.22);
        private static final double HUMAN_TAG_HEIGHT = Units.inchesToMeters(27.38);
        public static final double GRID_BARRIER = Units.inchesToMeters(15);
        public static final double ALLOWABLE_ERROR = Units.inchesToMeters(3);

        public static final Pose3d TAG_1_POSE = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), GRID_TAG_HEIGHT, new Rotation3d(0, 0, 0));
        public static final Pose3d TAG_2_POSE = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.91), GRID_TAG_HEIGHT, new Rotation3d(0, 0, 0));
        public static final Pose3d TAG_3_POSE = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), GRID_TAG_HEIGHT, new Rotation3d(0, 0, 0));
        public static final Pose3d TAG_4_POSE = new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), HUMAN_TAG_HEIGHT, new Rotation3d(0, 0, 0));
        public static final Pose3d TAG_5_POSE = new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), HUMAN_TAG_HEIGHT, new Rotation3d(0, 0, Units.degreesToRadians(180)));
        public static final Pose3d TAG_6_POSE = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), GRID_TAG_HEIGHT, new Rotation3d(0, 0, Units.degreesToRadians(180)));
        public static final Pose3d TAG_7_POSE = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), GRID_TAG_HEIGHT, new Rotation3d(0, 0, Units.degreesToRadians(180)));
        public static final Pose3d TAG_8_POSE = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), GRID_TAG_HEIGHT, new Rotation3d(0, 0, Units.degreesToRadians(180)));


    }
}
