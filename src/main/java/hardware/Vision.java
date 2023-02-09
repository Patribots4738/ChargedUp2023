package hardware;

import java.util.HashMap;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    public static final double rotDeadzone = 10;

    public static final double xDeadZone = 0.1;

    public static final double allignmentSpeed = 0.5;
    
    public static final double allignmentRotSpeed = 0.05;


    private final PhotonCamera camera = new PhotonCamera("Patribots4738");

    private final HashMap<String, Double> tagInfo = new HashMap<String, Double>();

    private boolean hasTargets;


    public Vision(){}


    /**
     * Converts the PhotonPipelineResult to an easier to use format
     * @return A HashMap<String, Double> representing the most visible AprilTag, null if no tag is visible
     */
    public HashMap<String, Double> pereodic(){

        // Get the most recent image from the camera and run the calculations
        // necessary to get AprilTag data
        PhotonPipelineResult result = camera.getLatestResult();

        // Make sure that the camera can see an AprilTag
        if (!result.hasTargets()){
            hasTargets = false;
            return null;
        }

        hasTargets = true;
        
        // Create the HashMap that represents the most visible tag
        
        
        // Get the best tag visible
        PhotonTrackedTarget target = result.getBestTarget();

        // Extract the position of the best tag
        Transform3d position = target.getBestCameraToTarget();
        
        // Add all the required info to the HashMap
        tagInfo.put("tagID", Double.valueOf(target.getFiducialId()));

        tagInfo.put("x", position.getX());

        tagInfo.put("y", position.getY());

        tagInfo.put("z", position.getZ());

        tagInfo.put("yaw", target.getYaw());

        tagInfo.put("pitch", target.getPitch());

        // Returns the HashMap
        return tagInfo;
    }

    public boolean hasTargets(){
        return this.hasTargets;
    }

    public Double getX(){
        return tagInfo.get("x");
    }

    public Double getY(){
        return tagInfo.get("y");
    }

    public Double getZ(){
        return tagInfo.get("z");
    }

    public Double getYaw(){
        return tagInfo.get("yaw");
    }

    public Double getPitch(){
        return tagInfo.get("pitch");
    }

    public int getTagID(){
        return tagInfo.get("tagID").intValue();
    }

    public Pose2d getPose(){
        return new Pose2d(getX(), getY(), new Rotation2d(getYaw()));
    }
}