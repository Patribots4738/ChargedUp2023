package calc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.util.DriverUI;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import java.io.IOException;
import java.util.Optional;
import calc.Constants.VisionConstants;

public class PhotonCameraUtil {

  public PhotonCamera cam1;
  public PhotonCamera cam2;
  public PhotonPoseEstimator cam1PoseEstimator;
  public PhotonPoseEstimator cam2PoseEstimator;
  public AprilTagFieldLayout aprilTagFieldLayout;

    public PhotonCameraUtil() {

        // Load the AprilTag field layout from the "resource file" at edu.wpi.first.apriltag.AprilTagFieldLayout
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println("AprilTag field layout not found!");
        }

        try {
            cam1 = new PhotonCamera(VisionConstants.CAMERA_1_NAME);
        } catch (Exception e) {
            System.out.println("Camera 1 (front) not found!");
        }

        try {
            cam2 = new PhotonCamera(VisionConstants.CAMERA_2_NAME);
        } catch (Exception e) {
            System.out.println("Camera 2 (back) not found!");
        }

        cam1PoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            cam1,
            VisionConstants.CAMERA_1_POSITION);

        cam2PoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            cam2,
            VisionConstants.CAMERA_2_POSITION);

    }

    // Look through both cameras and set the reference pose with
    // the camera that has the lowest ambiguity if both cameras can see a target, 
    // or,
    // the camera that can see a target if only one camera can see a target
    // only if it's ambiguity is lower than the threshold
    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
      // First, update the pose estimators
      Optional<EstimatedRobotPose> cam1Pose = cam1PoseEstimator.update();
      Optional<EstimatedRobotPose> cam2Pose = cam2PoseEstimator.update();
      // Then, get the current camera data
      Optional<PhotonCamera> cam1 = getCam1();
      Optional<PhotonCamera> cam2 = getCam2();
      // Make ambiguity doubles to compare the results
      double cam1Ambiguity = 1;
      double cam2Ambiguity = 1;

      // Get the ambiguity of each camera, if applicable
      if (cam1.isPresent()) {
        PhotonPipelineResult result = cam1.get().getLatestResult();
        if (result.hasTargets()) {
          cam1Ambiguity = result.getBestTarget().getPoseAmbiguity();
          DriverUI.hasTargets = true;
        }
      }
      if (cam2.isPresent()) {
        PhotonPipelineResult result = cam2.get().getLatestResult();
        if (result.hasTargets()) {
          cam2Ambiguity = result.getBestTarget().getPoseAmbiguity();
          DriverUI.hasTargets = true;
        }
      }
      
      // If both cameras have results, and both cameras can see a target, compare them
      if (cam1Pose.isPresent() && cam2Pose.isPresent()) {
        // Return the camera with the lower ambiguity (only if under the minimum threshold)
        // Recall that the lower the ambiguity, the more confident we are in the pose
        if (cam1Ambiguity < cam2Ambiguity && cam1Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
          return cam1Pose;
        } else if (cam2Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
          return cam2Pose;
        }
      }

      // If only the front camera has a result, and it can see a target, return it
      // * if it has low ambiguity
      if (cam1Pose.isPresent() && cam1Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
        return cam1Pose;
      } 

      // If only the back camera has a result, and it can see a target, return it
      // * if it has low ambiguity
      if (cam2Pose.isPresent() && cam2Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
        return cam2Pose;
      }
      DriverUI.hasTargets = false;
      // return a null Optional<EstimatedRobotPose> if no camera can see a target
      return Optional.empty();
    }

    public Optional<PhotonCamera> getCam1() {
      return Optional.ofNullable(cam1);
    }

    public Optional<PhotonCamera> getCam2() {
      return Optional.ofNullable(cam2);
    }
}