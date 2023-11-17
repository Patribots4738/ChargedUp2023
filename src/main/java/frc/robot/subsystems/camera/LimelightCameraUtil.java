package frc.robot.subsystems.camera;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriverUI;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.VisionConstants;

import java.util.Optional;

public class LimelightCameraUtil extends SubsystemBase{
    
    public LimelightCamera cam1;
    public LimelightCamera cam2;

    public SwerveDrivePoseEstimator cam1PoseEstimator;
    public SwerveDrivePoseEstimator cam2PoseEstimator;

    public AprilTagFieldLayout aprilTagFieldLayout;
    
    public LimelightCameraUtil() {
        // Load the AprilTag field layout from the "resource file" at
        // edu.wpi.first.apriltag.AprilTagFieldLayout
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println("AprilTag field layout not found!");
        }

        try {
            cam1 = new LimelightCamera(VisionConstants.CAMERA_1_NAME);
        } catch (Exception e) {
            System.out.println("Camera 1 (front) not found!");
        }

        try {
            cam2 = new LimelightCamera(VisionConstants.CAMERA_2_NAME);
        } catch (Exception e) {
            System.out.println("Camera 2 (back) not found!");
        }

            //TODO: replace nulls
            cam1PoseEstimator = new SwerveDrivePoseEstimator(
                null, 
                null, 
                null, 
                null,
                null,
                null);

            //TODO: replace nulls
            cam2PoseEstimator = new SwerveDrivePoseEstimator(
                null, 
                null, 
                null, 
                null, 
                null, 
                null);
    }

    public Optional<Pose2d> getEstimatedRobotPose() {

        if (FieldConstants.IS_SIMULATION) {
            return Optional.empty();
        }

        // First, update the pose estimators

        // TODO: replace nulls with gyro angles and mod positions
        // TODO: use update or add vision mesurement with respect to tl and cl
        Optional<Pose2d> cam1Pose = 
            Optional.ofNullable( cam1PoseEstimator.update(null, null)) ;
        
            // Optional<Pose2d> cam2Pose = Optional.ofNullable(cam2PoseEstimator.update(null, null));

        // Then, get the current camera data
        Optional<LimelightCamera> cam1 = getCam1();
        // Optional<LimelightCamera> cam2 = getCam2();
        // Make ambiguity doubles to compare the results

        // double cam2Ambiguity = 1;

        // TODO: send the cam1 pose or send the cam1 Pose with other table settings?
        if (cam1.isPresent() && cam1.get().hasValidTarget()) {
            return cam1Pose;
        }

        DriverUI.hasTargets = false;
        // return a null Optional<EstimatedRobotPose> if no camera can see a target
        return Optional.empty();
    }

    public Optional<LimelightCamera> getCam1() {
        return Optional.ofNullable(cam1);
    }

    public Optional<LimelightCamera> getCam2() {
        return Optional.ofNullable(cam2);
    }

    @Override
    public void periodic() {
    }
}
