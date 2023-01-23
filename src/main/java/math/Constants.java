// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package math;

import edu.wpi.first.math.geometry.Translation2d;
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
public final class Constants 

{
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAngularSpeed = 6 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.toRadians(90);
    public static final double kFrontRightChassisAngularOffset = Math.toRadians(-180);
    public static final double kBackLeftChassisAngularOffset = Math.toRadians(0);
    public static final double kBackRightChassisAngularOffset = Math.toRadians(-90);

    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kXCorrectionP = 1;
    public static final double kXCorrectionI = 0;
    public static final double kXCorrectionD = 0;

    public static final double kYCorrectionP = 0.3;
    public static final double kYCorrectionI = 0;
    public static final double kYCorrectionD = 0;

    public static final double kRotationCorrectionP = -0.22;
    public static final double kRotationCorrectionI = 0;
    public static final double kRotationCorrectionD = 0;//.74;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  } 

  public static final class ArmConstants {

    public static final int kLowerArmMotorCanId = 10;
    public static final int kUpperArmMotorCanId = 9;

    // The length of the first pivot point to the second pivot point, in inches
    public static final double kLowerArmLength = 32;

    // The length of the second pivot point to the claw, in inches
    public static final double kUpperArmLength = 21;

    // The max reach of the bot horizontally, in inches
    public static final double kMaxReachX = 53;

    // The max reach of the bot vertically, in inches
    public static final double kMaxReachY = 53;

    // The gear ratio of the lower arm is 60:1
    public static final double kLowerArmGearRatio = 60;

    // The gear ratio of the upper arm is 25:7
    public static final double kUpperArmGearRatio = (25.0/7.0);

    // The factor that the encoder is multiplied by to get the actual position
    public static final double kLowerEncoderPositionFactor = (kLowerArmLength * Math.PI) / (kLowerArmGearRatio);
    public static final double kUpperEncoderPositionFactor = (kUpperArmLength * Math.PI) / (kUpperArmGearRatio);
    
    // The number of degrees that the upper arm can rotate
    public static final double kLowerFreedom = Units.degreesToRadians(150);
    
    // The number of degrees that the upper arm can rotate
    public static final double kUpperFreedom = Units.degreesToRadians(200);

    // The outputs used in the output range for the lower and upper arms
    public static final double kLowerMinOutput = -1;
    public static final double kLowerMaxOutput = 1;

    public static final double kUpperMinOutput = -1;
    public static final double kUpperMaxOutput = 1;

    /**
     * SysID values:
     * <p>
     * Lower:
     * P = 0.0032326
     * D = 0.0012612
     * <p>
     * Upper:
     * 
     */
    // PID values for the lower and upper arm
    public static final double kLowerP = 0.1;
    public static final double kLowerI = 0;
    public static final double kLowerD = 0.025;
    public static final double kLowerFF = 0;

    public static final double kUpperP = 1;
    public static final double kUpperI = 0;
    public static final double kUpperD = 0;
    public static final double kUpperFF = 0;

    // The below values are given from sysID calibration
    public static final double kSLower = 0.15999;
    public static final double kGLower = 0;//0.33966; (We don't like gravity)
    public static final double kVLower = 0.24941;
    public static final double kALower = 4.3455;

    public static final double kSUpper = 0;
    public static final double kGUpper = 0;// (We don't like gravity)
    public static final double kVUpper = 0;
    public static final double kAUpper = 0;



  }
}
