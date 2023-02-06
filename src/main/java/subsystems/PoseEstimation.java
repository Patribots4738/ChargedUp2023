package subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.ArrayList;

public class PoseEstimation {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    public PoseEstimation() {

        // Set up a test arena of two apriltags at the center of each driver station set
        final AprilTag tag18 =
                new AprilTag(
                        18,
                        new Pose3d(
                                new Pose2d(
                                        FieldConstants.length,
                                        FieldConstants.width / 2.0,
                                        Rotation2d.fromDegrees(180))));

        final AprilTag tag01 =
                new AprilTag(
                        01,
                        new Pose3d(
                                new Pose2d(
                                        0.0,
                                        FieldConstants.width / 2.0,
                                        Rotation2d.fromDegrees(0.0))));

        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag18);
        atList.add(tag01);

        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout aprilTagFieldLayout = null;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // Forward Camera
        photonCamera =
                new PhotonCamera(
                        VisionConstants
                                .cameraName); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        photonCamera, VisionConstants.robotToCam);
    }
}
