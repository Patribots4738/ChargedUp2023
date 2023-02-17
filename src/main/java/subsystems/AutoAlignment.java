package subsystems;

import auto.SwerveTrajectory;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import hardware.Swerve;
import math.Constants;
import math.Constants.AlignmentConstants;
import math.Constants.VisionConstants;

public class AutoAlignment {

    /**
     * A visual representation of the apriltag positions
     * / --------------------------------------------- \
     * 5                     |                       4
     * |                      |                        |
     * |                      |                        |
     * 6                      |                        3
     * |                      |                        |
     * 7                      |                        2
     * |                      |                        |
     * 8                      |                        1
     * \ --------------------------------------------- /
     */

    Swerve swerve;
    PhotonCameraPose photonCameraPose;
    private int tagID;
    private int coneOffset;

    public AutoAlignment(Swerve swerve) {
        this.swerve = swerve;
    }

    /**
     * Calibrate the odometry for the swerve
     *
     * @param visionTransform3d the position of the aprilTag relative to the bot
     */
    public void calibrateOdometry() {

      Optional<EstimatedRobotPose> result = photonCameraPose.getEstimatedRobotPose(swerve.getPoseEstimator().getEstimatedPosition());

      if (result.isPresent()) {

        EstimatedRobotPose camEstimatedPose = result.get();
        swerve.getPoseEstimator().addVisionMeasurement(camEstimatedPose.estimatedPose.toPose2d(), camEstimatedPose.timestampSeconds);

        setTagID(photonCameraPose.getPhotonCamera().getLatestResult().getBestTarget().getFiducialId());
        setConeOffset(0);
      }
    }

    public void moveToTag() {

        if (tagID == 0) {
            return;
        }

        Pose2d targetPose = getTagPos(tagID);

        Rotation2d heading = Rotation2d.fromDegrees(0);

        double coneOffsetLeft = VisionConstants.CONE_OFFSET_METERS;

        if (0 < tagID && tagID < 5) {
            coneOffsetLeft *= -1;
        } else if (4 < tagID && tagID < 9) {
            heading = Rotation2d.fromDegrees(180);
        }

        if (0 < tagID && tagID < 5) {
            targetPose = targetPose.plus(new Transform2d(new Translation2d(-AlignmentConstants.GRID_BARRIER, 0), Rotation2d.fromDegrees(0)));
        } else {
            targetPose = targetPose.plus(new Transform2d(new Translation2d(AlignmentConstants.GRID_BARRIER, 0), Rotation2d.fromDegrees(0)));
        }

        if (coneOffset == -1) {
            targetPose = targetPose.plus(new Transform2d(new Translation2d(0, coneOffsetLeft), Rotation2d.fromDegrees(0)));
        } else if (coneOffset == 1) {
            targetPose = targetPose.plus(new Transform2d(new Translation2d(0, -coneOffsetLeft), Rotation2d.fromDegrees(0)));
        }

        if (swerve.getPose().minus(targetPose).getTranslation().getNorm() < Units.inchesToMeters(AlignmentConstants.ALLOWABLE_ERROR)) {
            swerve.drive(0, 0, 0, false);
            SwerveTrajectory.trajectoryStatus = "done";
            return;
        }

        PathPlannerTrajectory tagTrajectory = PathPlanner.generatePath
        (
            new PathConstraints(0.1, 0.1),
            new PathPoint(swerve.getPose().getTranslation(),
                heading,
                swerve.getPose().getRotation()),
            new PathPoint(targetPose.getTranslation(),
                heading,
                targetPose.getRotation())
        );

        SwerveTrajectory.PathPlannerRunner(tagTrajectory, swerve, swerve.getPose());

        System.out.println("April Pose: " + getTagPos(tagID));
        System.out.println("Modified Target Pose: " + targetPose);
        System.out.println("Current Pose: " + swerve.getPose() + "\n\n");
    }

    public void moveRelative(double x, double y, double rotation) {

        Pose2d currentPose = swerve.getPose();

        Pose2d targetPose = new Pose2d(
                currentPose.getX() + x,
                currentPose.getY() + y,
                new Rotation2d(currentPose.getY() + rotation));

        State targetState = new State(0, 0, 0, targetPose, 0);

        ChassisSpeeds speeds = SwerveTrajectory.HDC.calculate(
                currentPose,
                targetState,
                targetPose.getRotation());

        swerve.drive(speeds.vxMetersPerSecond * 0.25,
                speeds.vyMetersPerSecond * 0.25,
                speeds.omegaRadiansPerSecond, false);

    }

    private Pose2d getTagPos(int tagID) {
        double tagX = 0.0;
        double tagY = 0.0;
        double tagZ = 0.0;
        Rotation2d rotation = Rotation2d.fromRadians(0);

        switch (tagID) {
            case 1:
                tagX = Constants.AlignmentConstants.TAG_1_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_1_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_1_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_1_POSE.getRotation().getZ());
                break;

            case 2:
                tagX = Constants.AlignmentConstants.TAG_2_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_2_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_2_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_2_POSE.getRotation().getZ());
                break;

            case 3:
                tagX = Constants.AlignmentConstants.TAG_3_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_3_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_3_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_3_POSE.getRotation().getZ());
                break;

            case 4:
                tagX = Constants.AlignmentConstants.TAG_4_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_4_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_4_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_4_POSE.getRotation().getZ());
                break;

            case 5:
                tagX = Constants.AlignmentConstants.TAG_5_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_5_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_5_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_5_POSE.getRotation().getZ());
                break;

            case 6:
                tagX = Constants.AlignmentConstants.TAG_6_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_6_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_6_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_6_POSE.getRotation().getZ());
                break;

            case 7:
                tagX = Constants.AlignmentConstants.TAG_7_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_7_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_7_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_7_POSE.getRotation().getZ());
                break;

            case 8:
                tagX = Constants.AlignmentConstants.TAG_8_POSE.getX();
                tagY = Constants.AlignmentConstants.TAG_8_POSE.getY();
                tagZ = Constants.AlignmentConstants.TAG_8_POSE.getZ();
                rotation = Rotation2d.fromRadians(Constants.AlignmentConstants.TAG_8_POSE.getRotation().getZ());
                break;
        }
        return new Pose2d(tagX, tagY, rotation);
    }

    public int getTagID() {
        return tagID;
    }

    public void setTagID(int tagID) {
        this.tagID = tagID;
    }

    public int getConeOffset() {
        return coneOffset;
    }

    public void setConeOffset(int coneOffset) {

      // Pan the coneOffset to the next tag if it is able to do so
      // It cannot do so if there is no grid in the desired direction
      if (coneOffset < -1) {
        if (tagID == 2 || tagID == 3) {
          this.tagID--;
          this.coneOffset = 1;
        }
        else if (tagID == 6 || tagID == 7) {
          this.tagID++;
          this.coneOffset = 1;
        }
      }
      else if (coneOffset > 1) {
        if (tagID == 1 || tagID == 2) {
          this.tagID++;
          this.coneOffset = -1;
        }
        else if (tagID == 7 || tagID == 8) {
          this.tagID--;
          this.coneOffset = -1;
        }
      }

      this.coneOffset = MathUtil.clamp(coneOffset, -1, 1);
    }

}