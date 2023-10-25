package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmLogger {

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

    private String root = "Arm";
    private Mechanism2d armDesired2D = new Mechanism2d(0, 0);
    private double[] armDesired3D = new double[3];
    private double[] armActual3D = new double[3];
    

    public ArmLogger() {}

    public void update(ArmLogs logs) {
        this.armDesired2D = logs.armDesired2D;
        this.armDesired3D = logs.armDesired3D;
        this.armActual3D = logs.armActual3D;

        log();
    }

    private void log() {
        logArmDesired2D();
        logArmDesired3D();
        logArmActual3D();
    }

    private void logArmDesired2D() {
        SmartDashboard.putData(root+"/"+"ArmDesired2D", this.armDesired2D);
    }
    private void logArmDesired3D() {
        SmartDashboard.putNumberArray(root+"/"+"ArmDesired3D", this.armDesired3D);
    }
    private void logArmActual3D() {
        SmartDashboard.putNumberArray(root+"/"+"ArmActual3D", this.armActual3D);
    }
}
