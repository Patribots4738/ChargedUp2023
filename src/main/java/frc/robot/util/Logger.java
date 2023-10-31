package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {
    
    public Logger() { }

    // Arm Logging stuff
    public static class ArmLog {
        public Mechanism2d armDesired2D;
        public double[] armDesired3D;
        public double[] armActual3D;

        public ArmLog(ArmLogBuilder armLogBuilder) {
            this.armDesired2D = armLogBuilder.armDesired2D;
            this.armDesired3D = armLogBuilder.armDesired3D;
            this.armActual3D = armLogBuilder.armActual3D;
        }
    }
    public static class ArmLogBuilder {

        public Mechanism2d armDesired2D = new Mechanism2d(0, 0);
        public double[] armDesired3D = new double[3];
        public double[] armActual3D = new double[3];

        public ArmLogBuilder() { }

        public ArmLogBuilder addArmDesired2D(Mechanism2d armDesired2D) {
            this.armDesired2D = armDesired2D;
            return this;
        }

        public ArmLogBuilder addArmDesired3D(double[] armDesired3D) {
            this.armDesired3D = armDesired3D;
            return this;
        }

        public ArmLogBuilder addArmActual3D(double[] armActual3D) {
            this.armActual3D = armActual3D;
            return this;
        }

        public ArmLog build() {
            return new ArmLog(this);
        }
    }
    public static class ArmLogger {

        private static Mechanism2d armDesired2D = new Mechanism2d(0, 0);
        private static double[] armDesired3D = new double[3];
        private static double[] armActual3D = new double[3];
    
        public static void update(ArmLog armLog) {
            armDesired2D = armLog.armDesired2D;
            armDesired3D = armLog.armDesired3D;
            armActual3D = armLog.armActual3D;
    
            log();
        }
    
        private static void log() {
            logArmDesired2D();
            logArmDesired3D();
            logArmActual3D();
        }
    
        private static void logArmDesired2D() {
            SmartDashboard.putData("Arm/ArmDesired2D", armDesired2D);
        }
        private static void logArmDesired3D() {
            SmartDashboard.putNumberArray("Arm/ArmDesired3D", armDesired3D);
        }
        private static void logArmActual3D() {
            SmartDashboard.putNumberArray("Arm/ArmActual3D", armActual3D);
        }
    }
    
    // Swerve Logging stuff
    public static class SwerveLog {
        public double robotRotation;
        public double[] desiredModuleStates;
        public double[] realModuleStates;
        public double[] pose3d;
    
        public SwerveLog(SwerveLogBuilder swerveLogBuilder) {
            this.robotRotation = swerveLogBuilder.robotRotation;
            this.desiredModuleStates = swerveLogBuilder.desiredModuleStates;
            this.realModuleStates = swerveLogBuilder.realModuleStates;
            this.pose3d = swerveLogBuilder.pose3d;
        }
    }
    public static class SwerveLogBuilder {
            
            public double robotRotation;
            public double[] desiredModuleStates = new double[4];
            public double[] realModuleStates = new double[4];
            public double[] pose3d = new double[3];
    
            public SwerveLogBuilder() { }
    
            public SwerveLogBuilder addRobotRotation(double robotRotation) {
                this.robotRotation = robotRotation;
                return this;
            }
    
            public SwerveLogBuilder addDesiredModuleStates(double[] desiredModuleStates) {
                this.desiredModuleStates = desiredModuleStates;
                return this;
            }
    
            public SwerveLogBuilder addRealModuleStates(double[] realModuleStates) {
                this.realModuleStates = realModuleStates;
                return this;
            }
    
            public SwerveLogBuilder addPose3d(double[] pose3d) {
                this.pose3d = pose3d;
                return this;
            }
    
            public SwerveLog build() {
                return new SwerveLog(this);
            }

    }
    public static class SwerveLogger {
    
        private static double robotRotation = 0;
        private static double[] desiredModuleStates = new double[4];
        private static double[] realModuleStates = new double[4];
        private static double[] pose3d = new double[3];
    
        public static void update(SwerveLog logs) {
            robotRotation = logs.robotRotation;
            desiredModuleStates = logs.desiredModuleStates;
            realModuleStates = logs.realModuleStates;
            pose3d = logs.pose3d;
    
            log();
        }
    
        public static void log() {
            logRealStates();
            logDesiredStates();
            logRobotRotation();
            logPose3d();
        }
    
        private static void logPose3d() {
            SmartDashboard.putNumberArray("Robot/Pose3d", pose3d);
        }
        private static void logRobotRotation() {
            SmartDashboard.putNumber("Robot/RobotRotation", robotRotation);
        }
        private static void logDesiredStates() {
            SmartDashboard.putNumberArray("Swerve/DesiredStates", desiredModuleStates);
        }
        private static void logRealStates() {
            SmartDashboard.putNumberArray("Swerve/RealStates", realModuleStates);
        }
    
    }
}
