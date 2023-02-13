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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import math.Constants;
import org.photonvision.EstimatedRobotPose;
import subsystems.PhotonCameraPose;

import java.util.Optional;

public class Swerve extends SubsystemBase {
  private SwerveDrivePoseEstimator poseEstimator;
  private MAXSwerveModule[] mSwerveMods;
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private PhotonCameraPose photonPose = new PhotonCameraPose();
  ;
  private final Field2d field = new Field2d();

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
          Constants.DriveConstants.kFrontLeftDrivingCanId,
          Constants.DriveConstants.kFrontLeftTurningCanId,
          Constants.DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
          Constants.DriveConstants.kFrontRightDrivingCanId,
          Constants.DriveConstants.kFrontRightTurningCanId,
          Constants.DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
          Constants.DriveConstants.kRearLeftDrivingCanId,
          Constants.DriveConstants.kRearLeftTurningCanId,
          Constants.DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
          Constants.DriveConstants.kRearRightDrivingCanId,
          Constants.DriveConstants.kRearRightTurningCanId,
          Constants.DriveConstants.kBackRightChassisAngularOffset);

  public Swerve() {
    zeroHeading();

    mSwerveMods = new MAXSwerveModule[]{
            m_frontLeft,
            m_frontRight,
            m_rearLeft,
            m_rearRight
    };

    poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics,
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


    SmartDashboard.putData("Field", field);
  }

  public void drive(Pose2d translation, boolean fieldRelative) {

    double xSpeed = translation.getX();
    xSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;

    double ySpeed = translation.getY();
    ySpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;

    double rotSpeed = translation.getRotation().getRotations();
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

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            Constants.DriveConstants.kMaxSpeedMetersPerSecond);

    for (int modNum = 0; modNum < mSwerveMods.length; modNum++) {
      mSwerveMods[modNum].setDesiredState(desiredStates[modNum]);
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

  public void updateOdometry() {

    poseEstimator.update(getYaw(), getModulePositions());

    Optional<EstimatedRobotPose> result = photonPose.getEstimatedRobotPose(poseEstimator.getEstimatedPosition());

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

//    public SwerveModuleState[] getModuleStates() {
//
//        SwerveModuleState[] states = new SwerveModuleState[4];
//        for (int modNum = 0; modNum < mSwerveMods.length; modNum++) {
//            states[modNum] = mSwerveMods[modNum].getState();
//        }
//        return states;
//
//    }

  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int modNum = 0; modNum < mSwerveMods.length; modNum++) {
      positions[modNum] = mSwerveMods[modNum].getPosition();
    }
    return positions;

  }

  /**
   * Resets the gyro to a heading of 0
   */
  public void zeroHeading() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    return (Constants.DriveConstants.kGyroReversed) ?
        Rotation2d.fromDegrees(360 - gyro.getAngle()) :
        Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void periodic() {

    updateOdometry();

    for (int modNum = 0; modNum < mSwerveMods.length; modNum++) {
      SmartDashboard.putNumber("Mod " + modNum + " Angle", mSwerveMods[modNum].getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + modNum + " Velocity", mSwerveMods[modNum].getState().speedMetersPerSecond);
    }
    SmartDashboard.putNumber("Heading", getYaw().getDegrees());
  }

  public void setBrakeMode() {
    for (MAXSwerveModule mSwerveMod : mSwerveMods) {
      mSwerveMod.setBrakeMode();
    }
  }

  public void resetEncoders() {
    for (MAXSwerveModule mSwerveMod : mSwerveMods) {
      mSwerveMod.resetEncoders();
    }
  }
}