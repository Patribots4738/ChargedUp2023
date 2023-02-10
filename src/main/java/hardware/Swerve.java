// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import math.Constants.DriveConstants;

public class Swerve { 
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
    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            new SwerveModulePosition[]{
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    /**
     * Creates a new DriveSubsystem.
     */
    public Swerve() {
    }

    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    // Get the odometry object
    public SwerveDriveOdometry getOdometry() {
        return m_odometry;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
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
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, Rotation2d.fromDegrees(m_gyro.getAngle()))
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate();
    }

    public void toggleSpeed() {
      this.speedMultiplier = (this.speedMultiplier == 1) ? 0.1 : 1;
    }

    /**
     * Sets the coast mode for the drive motors.
     * This is useful for when the robot is disabled
     * So we can freely move the robot around
     */
    public void setCoastMode() {
        m_frontLeft.setCoastMode();
        m_frontRight.setCoastMode();
        m_rearLeft.setCoastMode();
        m_rearRight.setCoastMode();
    }

    /**
     * Sets the brake mode for the drive motors.
     * This is useful for when the robot is enabled
     * So we can stop the robot quickly
     * (This is the default mode)
     */
    public void setBrakeMode() {
        m_frontLeft.setBrakeMode();
        m_frontRight.setBrakeMode();
        m_rearLeft.setBrakeMode();
        m_rearRight.setBrakeMode();
    }
}
