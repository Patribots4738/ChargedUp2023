/*
 * See: https://github.com/commodores/ChargedUpCode
 * for referenced code
 */
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import math.Constants.DriveConstants;
import org.photonvision.EstimatedRobotPose;

import subsystems.AutoAlignment;
import subsystems.PhotonCameraPose;

import java.util.Optional;

public class Swerve {
    private SwerveDrivePoseEstimator poseEstimator;
    
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


    // Odometry class for tracking robot pose
    //  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    //      DriveConstants.DRIVE_KINEMATICS,
    //      Rotation2d.fromDegrees(gyro.getAngle()),
    //      new SwerveModulePosition[]{
    //          m_frontLeft.getPosition(),
    //          m_frontRight.getPosition(),
    //          m_rearLeft.getPosition(),
    //          m_rearRight.getPosition()
    //      });

    /**
     * Creates a new DriveSubsystem.
     */
    public Swerve() {

        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                new MatBuilder<>(
                        Nat.N3(),
                        Nat.N1()).fill(0.1, 0.1, 0.1),// State measurement
                // standard deviations.
                // X, Y, theta.
                new MatBuilder<>(
                        Nat.N3(),
                        Nat.N1()).fill(1.25, 1.25, 1.25));// Vision measurement
        // standard deviations.
        // X, Y, theta.);

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
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

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

    public void addVisionMeasurement(Optional<EstimatedRobotPose> result) {
        Optional<EstimatedRobotPose> result = photonCameraPose.getEstimatedRobotPose(poseEstimator.getEstimatedPosition());

        if (result.isPresent()) {

            EstimatedRobotPose camEstimatedPose = result.get();
            poseEstimator.addVisionMeasurement(camEstimatedPose.estimatedPose.toPose2d(), camEstimatedPose.timestampSeconds);
            
            field.getObject("Estimated Vision Position").setPose(camEstimatedPose.estimatedPose.toPose2d());

        } else {

            field.getObject("Estimated Vision Position").setPose(new Pose2d(-100, -100, new Rotation2d()));
        
        }

        field.getObject("Actual Pos").setPose(getPose());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
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
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getYaw() {
        return (DriveConstants.GYRO_REVERSED) ?
                Rotation2d.fromDegrees(360 - gyro.getAngle()) :
                Rotation2d.fromDegrees(gyro.getAngle());
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
        return gyro.getRate();
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