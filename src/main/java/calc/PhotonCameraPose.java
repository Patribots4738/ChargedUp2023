package calc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import java.util.Optional;
import calc.Constants.VisionConstants;

public class PhotonCameraPose {

  public PhotonCamera cam1;
  public PhotonCamera cam2;
  public PhotonPoseEstimator cam1PoseEstimator;
  public PhotonPoseEstimator cam2PoseEstimator;
  public AprilTagFieldLayout aprilTagFieldLayout;

    public PhotonCameraPose() {

        // Load the AprilTag field layout from the "resource file" at edu.wpi.first.apriltag.AprilTagFieldLayout
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            System.out.println("April tag field layout not found!");
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

      // If both cameras have results, and both cameras can see a target, compare them
      if (cam1Pose.isPresent() && cam2Pose.isPresent() && getCam1().isPresent() && getCam2().isPresent()) {
        // Get the ambiguity of each camera
        // Recall that the lower the ambiguity, the more confident we are in the pose
        double cam1Ambiguity = getCam1().get().getLatestResult().getBestTarget().getPoseAmbiguity();
        double cam2Ambiguity = getCam2().get().getLatestResult().getBestTarget().getPoseAmbiguity();
        // Return the camera with the lower ambiguity (only if under the minimum threshold)
        if (cam1Ambiguity < cam2Ambiguity && cam1Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
          return cam1Pose;
        } else if (cam2Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
          return cam2Pose;
        }
      }

      // If only the front camera has a result, and it can see a target, return it
      // * only if it has low ambiguity
      if (cam1Pose.isPresent() && getCam1().isPresent()) {
        
        double cam1Ambiguity = getCam1().get().getLatestResult().getBestTarget().getPoseAmbiguity();
        if (cam1Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
          return cam1Pose;
        }
      } 

      // If only the back camera has a result, and it can see a target, return it
      // * only if it has low ambiguity
      if (cam2Pose.isPresent() && getCam2().isPresent()) {
        
        double cam2Ambiguity = getCam2().get().getLatestResult().getBestTarget().getPoseAmbiguity();
        if (cam2Ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) {
          return cam2Pose;
        }
      }
      
      // return a null Optional<EstimatedRobotPose> if no camera can see a target
      return Optional.empty();

    }
3
    public Optional<PhotonCamera> getCam1() {
        return Optional.ofNullable(cam1);
    }

    public Optional<PhotonCamera> getCam2() {
      return Optional.ofNullable(cam2);
    }
}