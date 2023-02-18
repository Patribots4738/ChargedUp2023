// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package hardware;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import math.Constants.DriveConstants;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;


public class Swerve {

    private double speedMultiplier = 1;
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

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

    private final MAXSwerveModule[] swerveModules = new MAXSwerveModule[]{
            m_frontLeft,
            m_frontRight,
            m_rearLeft,
            m_rearRight
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
          DriveConstants.DRIVE_KINEMATICS,
          getYaw(),
          getModulePositions(),
          new Pose2d(),
          // Trust the information of the vision more
          // Nat.N1()).fill(0.1, 0.1, 0.1) --> trust more
          // Nat.N1()).fill(1.25, 1.25, 1.25) --> trust less
          new MatBuilder<>(
                  Nat.N3(),
                  Nat.N1()).fill(1.25, 1.25, 1.25),// State measurement
                  // standard deviations
                  // X, Y, theta
          new MatBuilder<>(
                  Nat.N3(),
                  Nat.N1()).fill(0.1, 0.1, 0.1)// Vision measurement
                  // standard deviations
                  // X, Y, theta
      );
  


    // Odometry class for tracking robot pose
    // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    //         DriveConstants.DRIVE_KINEMATICS,
    //         Rotation2d.fromDegrees(getTotalDegrees()),
    //         new SwerveModulePosition[]{
    //                 m_frontLeft.getPosition(),
    //                 m_frontRight.getPosition(),
    //                 m_rearLeft.getPosition(),
    //                 m_rearRight.getPosition()
    //         });

    /**
     * Creates a new DriveSubsystem.
     */
    public Swerve() {
      resetEncoders();
      zeroHeading();
      setBrakeMode();
    }

    public void periodic() {
        // Update the poseEstimator to account for the changes in the modules since the last loop
        poseEstimator.update(getYaw(), getModulePositions());

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            SmartDashboard.putNumber("Mod " + modNum + " Angle", swerveModules[modNum].getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + modNum + " Velocity", swerveModules[modNum].getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Heading", getYaw().getDegrees());
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {

        xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        rotSpeed *= DriveConstants.MAX_ANGULAR_SPEED;

        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getPose().getRotation())
                        : new ChassisSpeeds(ySpeed, xSpeed, rotSpeed));

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            swerveModules[modNum].setDesiredState(desiredStates[modNum]);
        }

    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
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

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's total degrees traveled from the start
     */
    public double getTotalDegrees() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees() 
                * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Rotation2d getYaw() {
        Rotation2d yaw = Rotation2d.fromDegrees(gyro.getAngle());

        if (DriveConstants.GYRO_REVERSED) {
            yaw = yaw.unaryMinus();
        }

        return yaw;
    }

    public void setBrakeMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setBrakeMode();
        }
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
     * Set the swerve wheels to point in an X direction
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }
}