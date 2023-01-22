// Refrenced from https://github.com/Stampede3630/2022-Code/blob/MK3Practice/src/main/java/frc/robot/AutoWaypoints.java
package auto;

import hardware.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AutoWaypoints implements Loggable {

    private static AutoWaypoints SINGLE_INSTANCE = new AutoWaypoints();

    public PathPlannerTrajectory squarePath;

    Swerve swerve;

    private double currentX;
    private double currentY;
 
    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(Swerve swerve) {
      this.swerve = swerve; 
      squarePath = PathPlanner.loadPath("Square", 3, 2.5);
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
    


    public void startPointRunner(String _startPoint) {
      if(_startPoint != "") {
        CurrentStartPoint = _startPoint;
      }
      if (CurrentStartPoint == "") {
        CurrentStartPoint = AutoPoses.values()[0].toString();
      }

    }

    public enum AutoState {
      BALL1TRANSITION(SINGLE_INSTANCE::intakeBall1, "SHOOT1TRANSITION"),
      SHOOT1TRANSITION(SINGLE_INSTANCE::shoot1, "BALL2TRANSITION"),
      BALL2TRANSITION(SINGLE_INSTANCE::intakeBall2, "BALL3TRANSITION"),
      BALL3TRANSITION(SINGLE_INSTANCE::intakeBall3, "SHOOT2TRANSITION"),
      SHOOT2TRANSITION(SINGLE_INSTANCE::shoot2, "SHOOT2TRANSITION");
      
      private Runnable action;
      private String nextState;

      AutoState(Runnable _action, String _nextState) {
        action = _action;
        nextState = _nextState;
      }

      public Runnable getAction() {
        return action;
      }

      public String getNextState() {
        return nextState;
      }
    }

    public void autoRunner(String _startingState) {
      if (_startingState != "" && StartingStateOverride) {
        CurrentState = _startingState;
        StartingStateOverride = false;
      } 
      
      if (CurrentState == "") {
        CurrentState = AutoState.values()[0].toString();
      }

      //if we made one round with the state, we have successfully initialized
      AutoState.valueOf(CurrentState).getAction().run();
      if (!StateHasInitialized) {StateHasInitialized = true;}
      if (StateHasFinished) {
        CurrentState = AutoState.valueOf(CurrentState).getNextState();
        StateHasFinished = false; 
        StateHasInitialized = false;
      }
    }

    private void intakeBall1() {
      double ballX = 7.801352311565382;
      double ballY = 1.6783324705889917;

      if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
        //Robot.INTAKE.intakeNow = true;
        StateHasFinished = true;
      }
    }

    private void intakeBall2() {
      double ballX = 5.278066252335036;
      double ballY = 2.0139989647067895;

      if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
        // Robot.INTAKE.intakeNow = true;
        StateHasFinished = true;
      }
    }

    private void intakeBall3() {
      double ballX = 1.2732177363088897;
      double ballY = 1.2153442028403045;

      if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
        // Robot.INTAKE.intakeNow = true;
        StateHasFinished = true;
      }
    }

    private void shoot1() {
      double posX = 7.882375258421403;
      double posY = 2.951550206897882;

      if (getDistance(currentX, currentY, posX, posY) < 0.5) {
        // Robot.INTAKE.shootNow = true;
        StateHasFinished = true;
      }
    }

    private void shoot2() {
      double posX = 7.581432884384757;
      double posY = 3.0325731537539022;

      if (getDistance(currentX, currentY, posX, posY) < 0.5) {
        // Robot.INTAKE.shootNow = true;
        StateHasFinished = true;
      }
    }

    // Just Pythagorean Theorem - uses current x and y positions
    private double getDistance(double X1, double Y1, double X2, double Y2) { 
      return Math.sqrt(Math.pow((X2 - X1), 2) + Math.pow((Y2 - Y1), 2));
    }
 
    @Config.ToggleButton(name = "Four Ball Auto", defaultValue = false)
    public void fbaStartButton(boolean _input, double thisX, double thisY, double thisRot) {
      if(_input) {
        _startPoint = "SQUARESTART";
        currentX = thisX;
        currentY = thisY;
        _input = false;
      }
    }
}