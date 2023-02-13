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

public class Swerve {
  private SwerveDrivePoseEstimator poseEstimator;
  private PhotonCameraPose photonPose = new PhotonCameraPose();
  private double speedMultiplier = 1;
  private final Field2d field = new Field2d();

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
  private PhotonCameraPose photonPose = new PhotonCameraPose();
  // Odometry class for tracking robot pose
//  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
//      DriveConstants.DRIVE_KINEMATICS,
//      Rotation2d.fromDegrees(m_gyro.getAngle()),
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
  }

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

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getOdometry().getPoseMeters().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

    setModuleStates(swerveModuleStates);
  }

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);

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

  public void updateOdometry(){

        poseEstimator.update(getYaw(),getModulePositions());

        Optional<EstimatedRobotPose> result=photonPose.getEstimatedRobotPose(poseEstimator.getEstimatedPosition());

        if(result.isPresent()){

        EstimatedRobotPose camPose=result.get();
        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),camPose.timestampSeconds);
        field.getObject("Estimated Vision Position").setPose(camPose.estimatedPose.toPose2d());

        } else {
        field.getObject("Estimated Vision Position").setPose(new Pose2d(-100,-100,new Rotation2d()));
        }

        field.getObject("Actual Pos").setPose(getPose());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
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
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
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

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public void toggleSpeed() {
    this.speedMultiplier = (this.speedMultiplier == 1) ? 0.1 : 1;
  }
}