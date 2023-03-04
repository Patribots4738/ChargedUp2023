// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package hardware;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import calc.Constants.DriveConstants;
import org.ejml.simple.SimpleMatrix;

public class Swerve implements Loggable{

    @Log
    private double yaw = 0;

    @Log
    public double pitch = 0;

    @Log
    public double roll = 0;

    @Log
    public double dotProduct = 0;

    @Log
    public double crossProduct = 0;


    private double speedMultiplier = 1;

    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // The gyro sensor
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final MAXSwerveModule[] swerveModules = new MAXSwerveModule[]{
            m_frontLeft,
            m_frontRight,
            m_rearLeft,
            m_rearRight
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.DRIVE_KINEMATICS,
        getGyroAngle(),
        getModulePositions(),
        new Pose2d(),
        // Trust the information of the vision more
        // Nat.N1()).fill(0.1, 0.1, 0.1) --> trust more
        // Nat.N1()).fill(1.25, 1.25, 1.25) --> trust less
        // Notice that the theta on the vision is very large,
        // and the state measurement is very small.
        // This is because we assume that the IMU is very accurate.
        // You can visualize these graphs working together here: https://www.desmos.com/calculator/a0kszyrwfe
        new MatBuilder<>(
            Nat.N3(),
            Nat.N1()).fill(0.1, 0.1, 0.05),
                // State measurement
                // standard deviations
                // X, Y, theta
        new MatBuilder<>(
                Nat.N3(),
                Nat.N1()).fill(0.9, 0.9, 5)
                // Vision measurement
                // standard deviations
                // X, Y, theta
    );

    /**
     * Creates a new DriveSu1stem.
     */
    public Swerve() {
        resetEncoders();
        zeroHeading();
        setBrakeMode();
    }

    public void periodic() {
        // Update the odometry in the periodic block
        poseEstimator.update(getGyroAngle(), getModulePositions());
        getPitch();
        getRoll();

        dotProduct = (pitch + (pitch > 0 ? -180 : 180)) + (roll + ((roll > 0) ? -180 : 180));
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        // Adjust input based on max speed
        xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier;
        ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier;
        rotSpeed *= DriveConstants.MAX_ANGULAR_SPEED * speedMultiplier;

        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, poseEstimator.getEstimatedPosition().getRotation())
                        : new ChassisSpeeds(ySpeed, xSpeed, rotSpeed));

        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setWheelsX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public void setWheelsUp() {
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(getYaw())));
  }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                getYaw(),
                getModulePositions(),
                pose);
    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            states[modNum] = swerveModules[modNum].getState();
        }
        return states;

    }

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            positions[modNum] = swerveModules[modNum].getPosition();
        }
        return positions;

    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }
    
    public Rotation2d getGyroAngle() {

      // gyro.setYawAxis(IMUAxis.kZ);
      
      Rotation2d yawRotation2d = Rotation2d.fromDegrees(gyro.getAngle());
      
      if (DriveConstants.GYRO_REVERSED) {
          yawRotation2d = yawRotation2d.unaryMinus();
      }

      this.yaw = yawRotation2d.getDegrees();

      return yawRotation2d;
    }

    public Rotation2d getYaw() { return this.getPose().getRotation(); }

    public Rotation2d getPitch() {

      Rotation2d pitchRotation2d = Rotation2d.fromDegrees(gyro.getYComplementaryAngle());

      this.pitch = pitchRotation2d.getDegrees();
      
      return pitchRotation2d;
      
    }

    public Rotation2d getRoll() {

      Rotation2d rollRotation2d = Rotation2d.fromDegrees(gyro.getYComplementaryAngle());

      this.roll = rollRotation2d.getDegrees();

      return rollRotation2d;
      
    }

    public double getTilt() {

      Rotation3d gyroRotation3d = new Rotation3d(getRoll().getRadians(), getPitch().getRadians(), getYaw().getRadians());

      Quaternion gyroQuaternion = gyroRotation3d.getQuaternion();
      // Multiply the quaternion by the UP direction as a vector to normalize it to UP
      // In english: this basically converts the angles to be relative to the charging station
      double num = gyroQuaternion.getX() + gyroQuaternion.getX();
      double num2 = gyroQuaternion.getY() + gyroQuaternion.getY();
      double num3 = gyroQuaternion.getZ() + gyroQuaternion.getZ();
      double num4 = gyroQuaternion.getW() * num;
      double num5 = gyroQuaternion.getW() * num2;
      double num6 = gyroQuaternion.getW() * num3;
      double num7 = gyroQuaternion.getX() * num;
      double num8 = gyroQuaternion.getX() * num2;
      double num9 = gyroQuaternion.getX() * num3;
      double num10 = gyroQuaternion.getY() * num2;
      double num11 = gyroQuaternion.getY() * num3;
      double num12 = gyroQuaternion.getZ() * num3;
      double num13 = 1.0 - num10 - num12;
      double num14 = num8 - num6;
      double num15 = num9 + num5;
      double num16 = num8 + num6;
      double num17 = 1.0 - num7 - num12;
      double num18 = num11 - num4;
      double num19 = num9 - num5;
      double num20 = num11 + num4;
      double num21 = 1.0 - num7 - num10;

      Vector multiplyOutput = new Vector(new SimpleMatrix(new double[][] {{0 * num13 + 1 * num14 + 0 * num15}, {0 * num16 + 1 * num17 + 0 * num18}, {0 * num19 + 1 * num20 + 0 * num21}}));

      double dotProduct = multiplyOutput.dot(new Vector(new SimpleMatrix(new double[][] {{0}, {1}, {0}})));

      return (Math.acos(MathUtil.clamp(dotProduct, -1, 1))) * Math.signum(getPitch().getRadians());
    }

    public void resetEncoders() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.resetEncoders();
        }
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public void toggleSpeed() {
        this.speedMultiplier = (this.speedMultiplier == 1) ? 0.1 : 1;
    }

    /**
     * Sets the brake mode for the drive motors.
     * This is useful for when the robot is enabled
     * So we can stop the robot quickly
     * (This is the default mode)
     */
    public void setBrakeMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setBrakeMode();
        }
    }

    /**
     * Sets the coast mode for the drive motors.
     * This is useful for when the robot is disabled
     * So we can freely move the robot around
     */
    public void setCoastMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setCoastMode();
        }
    }


}
