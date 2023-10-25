package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveLogger {

    public static final class SwerveLogs {
        public Pose2d robotPose;
        public double robotRotation;
        public double[] desiredModuleStates;
        public double[] realModuleStates;
        public double[] pose3d;

        public SwerveLogs(Pose2d robotPose, double robotRotaion, double[] desiredModuleStates,
            double[] realModuleStates, double[] pose3d){
                this.robotPose = robotPose;
                this.robotRotation = robotRotaion;
                this.desiredModuleStates = desiredModuleStates;
                this.realModuleStates = realModuleStates;
                this.pose3d = pose3d;
            }
    }

    private String root = "Swerve";

    private Field2d field = new Field2d();

    private Pose2d robotPose = new Pose2d();
    private double robotRotation = 0;
    private double[] desiredModuleStates = new double[4];
    private double[] realModuleStates = new double[4];
    private double[] pose3d = new double[3];

    public SwerveLogger() {}

    public void update(SwerveLogs logs) {
        this.robotPose = logs.robotPose;
        this.robotRotation = logs.robotRotation;
        this.desiredModuleStates = logs.desiredModuleStates;
        this.realModuleStates = logs.realModuleStates;
        this.pose3d = logs.pose3d;

        log();
    }

    public void log() {
        logField();
        logRealStates();
        logDesiredStates();
        logRobotRotation();
        logPose3d();
    }

    private void logPose3d() {
        SmartDashboard.putNumberArray(root+"/"+"Pose3d", this.pose3d);
    }
    private void logRobotRotation() {
        SmartDashboard.putNumber(root+"/"+"RobotRotation", this.robotRotation);
    }
    private void logDesiredStates() {
        SmartDashboard.putNumberArray(root+"/"+"DesiredStates", this.desiredModuleStates);
    }
    private void logRealStates() {
        SmartDashboard.putNumberArray(root+"/"+"RealStates", realModuleStates);
    }
    private void logField() {
        field.setRobotPose(this.robotPose);
    }

    public void makeFeild() {
        SmartDashboard.putData(root+"/"+"Field", field);
    }

}
