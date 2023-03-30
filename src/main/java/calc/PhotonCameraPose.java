package calc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import java.util.Optional;
import calc.Constants.VisionConstants;

public class PhotonCameraPose {

    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
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
            photonCamera = new PhotonCamera(VisionConstants.CAMERA_1_NAME);
        } catch (Exception e) {
            System.out.println("Camera not found!");
            return;
        }

        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                photonCamera,
                VisionConstants.CAMERA_1_POSITION);

    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return photonPoseEstimator.update();
    }

    public Optional<PhotonCamera> getPhotonCamera() {

        return Optional.ofNullable(photonCamera);

    }
}