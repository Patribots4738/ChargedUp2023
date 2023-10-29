package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {
    
    public Logger() { }
    
    public static final class ArmLogs {
        public Mechanism2d armDesired2D;
        public double[] armDesired3D;
        public double[] armActual3D;

        public ArmLogs(Mechanism2d armDesired2D,
                double[] armDesired3D,
                double[] armActual3D) {
            this.armDesired2D = armDesired2D;
            this.armDesired3D = armDesired3D;
            this.armActual3D = armActual3D;
        }
    }
    
    public static class ArmLogger {
    
        private static String root = "Arm";
        private static Mechanism2d armDesired2D = new Mechanism2d(0, 0);
        private static double[] armDesired3D = new double[3];
        private static double[] armActual3D = new double[3];
        
    
        public ArmLogger() {}
    
        public static void update(ArmLogs logs) {
            armDesired2D = logs.armDesired2D;
            armDesired3D = logs.armDesired3D;
            armActual3D = logs.armActual3D;
    
            log();
        }
    
        private static void log() {
            logArmDesired2D();
            logArmDesired3D();
            logArmActual3D();
        }
    
        private static void logArmDesired2D() {
            SmartDashboard.putData(root+"/"+"ArmDesired2D", armDesired2D);
        }
        private static void logArmDesired3D() {
            SmartDashboard.putNumberArray(root+"/"+"ArmDesired3D", armDesired3D);
        }
        private static void logArmActual3D() {
            SmartDashboard.putNumberArray(root+"/"+"ArmActual3D", armActual3D);
        }
    }

    public static class SwerveLogs {
            Pose2d robotPose = new Pose2d();
            double robotRotation;
            double[] desiredModuleStates;
            double[] realModuleStates;
            double[] pose3d;
    
            public SwerveLogs(Pose2d robotPose, double robotRotaion, double[] desiredModuleStates,
                double[] realModuleStates, double[] pose3d){
                    this.robotPose = robotPose;
                    this.robotRotation = robotRotaion;
                    this.desiredModuleStates = desiredModuleStates;
                    this.realModuleStates = realModuleStates;
                    this.pose3d = pose3d;
                }
        }
    
    public static class SwerveLogger {

        
        private static String root = "Swerve";
    
        private static Field2d field = new Field2d();
    
        private static Pose2d robotPose = new Pose2d();
        private static double robotRotation = 0;
        private static double[] desiredModuleStates = new double[4];
        private static double[] realModuleStates = new double[4];
        private static double[] pose3d = new double[3];
    
        public SwerveLogger() {}
    
        public static void update(SwerveLogs logs) {
            robotPose = logs.robotPose;
            robotRotation = logs.robotRotation;
            desiredModuleStates = logs.desiredModuleStates;
            realModuleStates = logs.realModuleStates;
            pose3d = logs.pose3d;
    
            log();
        }
    
        public static void log() {
            logField();
            logRealStates();
            logDesiredStates();
            logRobotRotation();
            logPose3d();
        }
    
        private static void logPose3d() {
            SmartDashboard.putNumberArray(root+"/"+"Pose3d", pose3d);
        }
        private static void logRobotRotation() {
            SmartDashboard.putNumber(root+"/"+"RobotRotation", robotRotation);
        }
        private static void logDesiredStates() {
            SmartDashboard.putNumberArray(root+"/"+"DesiredStates", desiredModuleStates);
        }
        private static void logRealStates() {
            SmartDashboard.putNumberArray(root+"/"+"RealStates", realModuleStates);
        }
        private static void logField() {
            field.setRobotPose(robotPose);
        }
    
        public static void makeFeild() {
            SmartDashboard.putData(root+"/"+"Field", field);
        }
    
        public static Pose2d getFieldRobotPose() {
            return field.getRobotPose();
        }
    
    }
}
