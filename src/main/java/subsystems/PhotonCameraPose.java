package subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import math.Constants.PoseEstimationConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

public class PhotonCameraPose {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;
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
}
