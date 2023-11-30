package frc.robot.subsystems.camera;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.camera.LightMode.SnapMode;

// https://github.com/NAHSRobotics-Team5667/2020-FRC/blob/master/src/main/java/frc/robot/utils/LimeLight.java
public class LimelightCamera extends SubsystemBase {

    private NetworkTable table;

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    private LightMode lightState = LightMode.OFF;

    public SendableChooser<Boolean> snapshotChooser = new SendableChooser<>();
    public SendableChooser<Boolean> lightChooser = new SendableChooser<>();

    public LimelightCamera() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        setLightState(LightMode.OFF);

        snapshotChooser.setDefaultOption("Disabled", false);
        snapshotChooser.addOption("Enabled", true);

        lightChooser.setDefaultOption("Off", false);
        lightChooser.addOption("On", true);
    }
    
    /**
     * @return ID of the primary in-view AprilTag
     */
    public double[] getTagID() {
        return table.getEntry("tid").getDoubleArray(new double[6]);
    }
    

    public Pose3d convertBotEntry(double[] entry) {
        return new Pose3d(
            entry[0], 
            entry[1], 
            entry[2],
            new Rotation3d(
                Units.degreesToRadians(entry[3]),
                Units.degreesToRadians(entry[4]),
                Units.degreesToRadians(entry[5])
            ));
    }

    /**
     * Robot transform in field-space. 
     * Translation (X,Y,Z) 
     * Rotation(Roll,Pitch,Yaw)
     * 
     * @param targetSpace if the bot pose should be send in targetSpace.
     * @return the robot pose
     */
    public Pose3d getBotPose(boolean targetSpace) {
        return convertBotEntry( getRawBotPose(targetSpace) );
    }

    public double[] getRawBotPose(boolean targetSpace) {
        return table.getEntry(
            (!targetSpace) ? "botpose" 
            : "botpose_targetspace").getDoubleArray(new double[6]);
    }

    /**
     * Robot transform in field-space (blue driverstation WPILIB origin). 
     * Translation (X,Y,Z) 
     * Rotation(Roll,Pitch,Yaw), 
     * total latency (cl+tl)
     * 
     * @return the robot pose with the blue driverstation WPILIB origin
     */
    @Deprecated
    public double[] getBluePose() {
        return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public Optional<Pose2d> getBluePose2d() {
        Pose2d result = convertBotEntry(getBluePose()).toPose2d();
        if (result.getX() == 0 && result.getY() == 0) {
            return Optional.empty();
        } else {
            return Optional.ofNullable(result);
        }
    }

    /**
     * Robot transform in field-space (red driverstation WPILIB origin). 
     * Translation (X,Y,Z) 
     * Rotation(Roll,Pitch,Yaw), 
     * total latency (cl+tl)
     * 
     * @return the robot pose with the red driverstation WPILIB origin
     */
    @Deprecated
    public double[] getRedPose() {
        return table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the camera in the coordinate 
     * system of the primary in-view AprilTag (array (6))
     * 
     * or 
     * 
     * the coordinate system of the robot (array (6))
     * 
     * @param targetSpace is weather or not the camera pose is returned as a targetSpace
     * @return the camera pose
     */
    public double[] getCameraPose(boolean targetSpace) {
        return table.getEntry((targetSpace) ? "camerapose_targetspace" : "camerapose_robotspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the primary in-view AprilTag
     * in the coordinate system of the Camera (array (6))
     * 
     * or 
     * 
     * the coordinate system of the Robot (array (6))
     * 
     * @param cameraSpace is weather or not the target pose is returned as a cameraSpace
     * @return the target pose
     */
    public double[] getTargetPose(boolean cameraSpace) {
        return table.getEntry((cameraSpace) ? "targetpose_cameraspace" : "targetpose_robotspace").getDoubleArray(new double[6]);
    }
    
    public boolean containsTagID(boolean cameraSpace, int tagID) {
        double[] targetPose = getTargetPose(cameraSpace);
        for (double pose : targetPose) {
            if (pose == tagID) { return true; }
        }
        return false;
    }

    /**
     * Are we currently tracking any potential targets
     * 
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        return (table.getEntry("tv").getDouble(0) == 0) ? false : true;
    }

    /**
     * Horizontal offset from crosshair to target
     * 
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
     *         degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getXAngle() {
        return tx.getDouble(0);
    }

    /**
     * Vertical offset from crosshair to target
     * 
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
     *         degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getYAngle() {
        return ty.getDouble(0);
    }

    /**
     * Get the area of the vision tracking box
     * 
     * @return Target Area (0% of image to 100% of image)
     */
    public double getArea() {
        return ta.getDouble(0);
    }

    /**
     * Rotation of the object
     * 
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    /**
     * Latency in ms of the pipeline
     * 
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     *         capture latency.
     */
    public double getPipelineLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    /**
     * Time between the end of the exposure of the middle row 
     * of the sensor to the beginning of the tracking pipeline.
     * 
     * @return Capture pipeline latency (s). 
     */
    public double getCaptureLatency() {
        return table.getEntry("cl").getDouble(0)/1000.0;
    }

    /**
     * Total latency (ms) of the entire pipeline (ms)
     * 
     * @return Total latency (ms)
     */
    public double getTotalLatency() {
        return getPipelineLatency() + getCaptureLatency();
    }

    /**
     * The length of the shortest side of the bounding box in pixels
     * 
     * @return Side length of shortest side of the fitted bounding box (pixels)
     */
    public double getShortLength() {
        return table.getEntry("tshort").getDouble(0);
    }

    /**
     * The length of the longest side of the bounding box in pixels
     * 
     * @return Side length of longest side of the fitted bounding box (pixels)
     */
    public double getLongLength() {
        return table.getEntry("tlong").getDouble(0);
    }

    /**
     * The length of the horizontal side of the box (0-320 pixels)
     * 
     * @return Horizontal side length of the rough bounding box (0 - 320 pixels)
     */
    public double getHorizontalLength() {
        return table.getEntry("thor").getDouble(0);
    }

    /**
     * The length of the vertical side of the box (0-320 pixels)
     * 
     * @return Vertical side length of the rough bounding box (0 - 320 pixels)
     */
    public double getVerticalLength() {
        return table.getEntry("tvert").getDouble(0);
    }

    /**
     * Returns the index of the current vision pipeline (0... 9)
     * 
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public int getPipeIndex() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    /**
     * The X-Coordinates of the tracked box
     * 
     * @return Number array of corner x-coordinates
     */
    public double[] getXCorners() {
        return table.getEntry("tcornx").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * The Y-Coordinates of the tracked box
     * 
     * @return Number array of corner y-coordinates
     */
    public double[] getYCorners() {
        return table.getEntry("tcorny").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * Sets the Lime Light LED's
     * 
     * @param mode - LightMode (On, Off, Blinking, or determined by the pipeline)
     */
    public void setLightState(LightMode mode) {
        lightState = mode;
        table.getEntry("ledMode").setNumber(lightState.getLedMode());
    }

    /**
     * Set the Lime Light Camera Mode
     * 
     * @param mode - VISION enables vision processing and decreases exposure, DRIVER
     *             disables vision processing and increases exposure
     */
    public void setCamMode(LightMode.CamMode mode) {
        table.getEntry("camMode").setNumber(mode.getMode());

    }

    /**
     * Sets the limelights current pipeline
     * 
     * @param pipeline The pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets the layout of the cameras viewed at 10.56.67.11:5800
     * 
     * @param streamMode - STANDARD is side by side, MAIN is Limelight big with
     *                   secondary camera in bottom right, SECONDARY is vice versa
     */
    public void setStreamMode(LightMode.StreamMode streamMode) {
        table.getEntry("stream").setNumber(streamMode.getMode());
    }

    /**
     * Allow the Lime Light to take snapshots so that it can be tuned off the field
     * 
     * @param mode DISABLED turns Snap Mode off, Enabled turns Snap Mode on
     */
    public void takeSnapshots(LightMode.SnapMode mode) {
        table.getEntry("snapshot").setNumber(mode.getMode());

    }

    /**
     * Toggle the current Lime Light LED's between on and off states
     */
    public void toggleLight() {
        lightState = (lightState == LightMode.ON) ? LightMode.OFF : LightMode.ON;
        setLightState(lightState);
    }

    /**
     * Turn the Lime Light LED's off
     */
    public void turnLightOff() {
        setLightState(LightMode.OFF);
    }

    /**
     * Turn the Lime Light LED's on
     */
    public void turnLightOn() {
        setLightState(LightMode.ON);
    }

    /**
     * Get the distance of the limelight target
     * 
     * @param h1 - The height of the limelight with respect to the floor
     * @param h2 - The height of the target
     * @param a1 - The mounting angle for the limelight
     * @param a2 - The angle between the limelight angle and the target
     * 
     * @return The distance between the robot and the limelight
     */
    public double getDistance(double h1, double h2, double a1, double a2) {
        return (h2 - h1) / Math.abs(Math.tan(Math.toRadians(a1) + Math.toRadians(a2)));
    }

    /**
     * Output diagnostics
     */
    public void outputTelemetry() {
        SmartDashboard.putBoolean("HasTarget", hasValidTarget());
        SmartDashboard.putNumber("Horizontal Offset", getXAngle());
        SmartDashboard.putNumber("Vertical Offset", getYAngle());
        SmartDashboard.putNumber("Area", getArea());
        SmartDashboard.putNumber("Skew", getSkew());
        SmartDashboard.putString("XCorners", Arrays.toString(getXCorners()));
        SmartDashboard.putString("YCorners", Arrays.toString(getYCorners()));
    }

    /**
     * Send the custom choosers to Shuffleboard
     */
    public void outputChoosers() {
        SmartDashboard.putData("Snapshot", snapshotChooser);
        SmartDashboard.putData("Light Mode", lightChooser);}

    public void updateChoosers() {
        if (lightChooser.getSelected())
            turnLightOn();
        else
            turnLightOff();

        if (snapshotChooser.getSelected())
            takeSnapshots(SnapMode.ENABLED);
        else
            takeSnapshots(SnapMode.DISABLED);
    }
}
