package subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import math.Constants.PoseEstimationConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.Optional;

public class PhotonCameraPose {
    
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout aprilTagFieldLayout;

    public PhotonCameraPose() {

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        photonCamera = new PhotonCamera(PoseEstimationConstants.CAMERA_NAME);

        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                photonCamera,
                PoseEstimationConstants.ROBOT_TO_CAMERA);

    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d pevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(pevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
    
    public void periodic() {
        System.out.println(photonCamera.getLatestResult().hasTargets());
    }

    public void updateOdometry() {

      photonPoseEstimator.update();

      Optional<EstimatedRobotPose> result = getEstimatedRobotPose(new Pose2d(0,0,Rotation2d.fromDegrees(0)));

      if (result.isPresent()) {
          EstimatedRobotPose camPose = result.get();   
          System.out.println("Position: " + camPose.estimatedPose.getTranslation() + 
                           "\nRotation:" + Rotation2d.fromRadians(camPose.estimatedPose.getRotation().getZ()) + "\n\n");
      }
    }
}
