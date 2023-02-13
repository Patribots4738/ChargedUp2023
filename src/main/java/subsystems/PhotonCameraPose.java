package subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import math.Constants.PoseEstimationConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import java.io.IOException;
import java.util.Optional;

public class PhotonCameraPose {
    
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout aprilTagFieldLayout;
    private Pose2d previousEstimatedRobotPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));
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
    
    // public void periodic() {
    //     System.out.println(photonCamera.getLatestResult().hasTargets());
    // }

    // public void updateOdometry() {



    //   if (photonCamera.getLatestResult().hasTargets()) {

    //       photonPoseEstimator.update();

    //       Optional<EstimatedRobotPose> result = getEstimatedRobotPose(previousEstimatedRobotPose);
    //       previousEstimatedRobotPose = result.get().estimatedPose.toPose2d();
    //       EstimatedRobotPose camPose = result.get();   

    //   }
    // }
}
