package hardware;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import math.Constants;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import subsystems.PhotonCameraPose;

public class NewSwerve extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator;
    private MAXSwerveModule[] mSwerveMods;
    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
    private PhotonCameraPose photonPose = new PhotonCameraPose();
    ;
    private Field2d field = new Field2d();

    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(Constants.DriveConstants.kFrontLeftDrivingCanId, Constants.DriveConstants.kFrontLeftTurningCanId, Constants.DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(Constants.DriveConstants.kFrontRightDrivingCanId, Constants.DriveConstants.kFrontRightTurningCanId, Constants.DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(Constants.DriveConstants.kRearLeftDrivingCanId, Constants.DriveConstants.kRearLeftTurningCanId, Constants.DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(Constants.DriveConstants.kRearRightDrivingCanId, Constants.DriveConstants.kRearRightTurningCanId, Constants.DriveConstants.kBackRightChassisAngularOffset);

    public NewSwerve() {
        zeroGyro();

        mSwerveMods = new MAXSwerveModule[]{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

        resetModulesToAbsolute();

        //swerveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, getYaw(), getModulePositions(), new Pose2d(), new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1), // State measurement
                // standard deviations.
                // X, Y, theta.
                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(1.25, 1.25, 1.25)); // Vision measurement
        // standard deviations.
        // X, Y, theta.);


        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Adjust input based on max speed
        double xSpeed = translation.getX();
        xSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        double ySpeed = translation.getY();
        ySpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        double rotSpeed = rotation;
        rotSpeed *= Constants.DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates =
                Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative ?
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed,
                                ySpeed,
                                rotSpeed,
                                getYaw()) :
                            new ChassisSpeeds(
                                    xSpeed,
                                    ySpeed,
                                    rotSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates,
                Constants.DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);


        //CONTINUE WORK FROM HERE DOWN 2/7/23
        for (MAXSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        //return swerveOdometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        //swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void updateOdometry() {
        poseEstimator.update(getYaw(), getModulePositions());

        Optional<EstimatedRobotPose> result = photonPose.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            field.getObject("Estimated Vision Position").setPose(camPose.estimatedPose.toPose2d());
        } else {
            field.getObject("Estimated Vision Position").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        field.getObject("Actual Pos").setPose(getPose());
        field.setRobotPose(poseEstimator.getEstimatedPosition());

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Resets the gyro to a heading of 0
     */
    public void zeroGyro() {
        zeroGyro(0);
    }

    /**
     * Resets the gyro to a given heading
     */
    public void zeroGyro(double angle) {
        gyro.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        //swerveOdometry.update(getYaw(), getModulePositions());
        updateOdometry();

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Heading", getYaw().getDegrees());
    }
}