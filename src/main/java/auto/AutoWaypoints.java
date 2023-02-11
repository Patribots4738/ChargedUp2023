// Refrenced from https://github.com/Stampede3630/2022-Code/blob/MK3Practice/src/main/java/frc/robot/AutoWaypoints.java
package auto;

import hardware.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AutoWaypoints implements Loggable {

    private static AutoWaypoints SINGLE_INSTANCE = new AutoWaypoints();

    public PathPlannerTrajectory squarePath;

    public PathPlannerTrajectory testTraj;

    Swerve swerve;

    private double currentX;
    private double currentY;

    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(Swerve swerve) {
        this.swerve = swerve;
        squarePath = PathPlanner.loadPath("Square", 3, 2.5);
        testTraj = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(swerve.getOdometry().getPoseMeters().getTranslation(), swerve.getOdometry().getPoseMeters().getRotation()),
                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45))
        );
        chooserBuilder();
    }

    public void autoPeriodic() {
        currentX = swerve.getPose().getX();
        currentY = swerve.getPose().getY();
    }

    @Log
    boolean StateHasFinished = false;
    @Log
    Boolean StateHasInitialized = false;
    @Log
    String CurrentState = "";
    boolean StartingStateOverride;
    SwerveDriveOdometry a_odometry;

    String _startPoint;
    String CurrentStartPoint;


    public SendableChooser<AutoPoses> m_autoChooser = new SendableChooser<>();

    public enum AutoPoses {
        SQUARESTART(5, 2, 0.00, "SQUARESTART");

        private double thisX;
        private double thisY;
        private double thisRot;
        private String thisStartPoint;

        AutoPoses(double _x, double _y, double _rot, String _startPoint) {
            thisX = _x;
            thisY = _y;
            thisRot = _rot;
            thisStartPoint = _startPoint;

        }

        public double getThisX() {
            return thisX;
        }

        public double getThisY() {
            return thisY;
        }

        public double getThisRot() {
            return thisRot;
        }

        public String getStartPoint() {
            return thisStartPoint;
        }

    }

    public void chooserBuilder() {

        for (AutoPoses myAutoPose : AutoPoses.values()) {
            SINGLE_INSTANCE.m_autoChooser.addOption(myAutoPose.toString(), myAutoPose);
        }

    }
}
    
