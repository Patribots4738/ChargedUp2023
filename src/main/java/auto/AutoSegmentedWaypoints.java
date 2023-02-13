// Referenced from https://github.com/Stampede3630/2022-Code/blob/0ad2aa434f50d8f5dc93e965809255f697dadffe/src/main/java/frc/robot/AutoSegmentedWaypoints.java#L81
package auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import math.Constants.ArmConstants;
import hardware.*;

public class AutoSegmentedWaypoints implements Loggable {

  Arm arm;
  Swerve swerve;

  public Waypoint[] HighFiveBallAutoWPs;
  public Waypoint[] SquareAutoWPs;
  public Waypoint[] chosenWaypoints;

  public int currentWaypointNumber = 0;

  public AutoPose chosenPath;
  public AutoPose[] myAutoContainer;

  public PathPlannerTrajectory seg1;
  public PathPlannerTrajectory seg2;
  public PathPlannerTrajectory seg3;
  public PathPlannerTrajectory seg4;

  @Log
  public double autoDelay;

  public boolean StateHasFinished = false;
  public Boolean StateHasInitialized = false;

  @Log(tabName = "CompetitionLogger", rowIndex = 0, columnIndex = 3, height = 1, width = 2)
  public SendableChooser<AutoPose> m_autoChooser = new SendableChooser<>();


  public AutoSegmentedWaypoints(Swerve swerve, Arm arm) {
    this.arm = arm;
    this.swerve = swerve;
  }

  public void init() {

    if (m_autoChooser.getSelected() == null) {
      chosenPath = myAutoContainer[0];
    } else {
      chosenPath = m_autoChooser.getSelected();
    }

    chosenWaypoints = chosenPath.thisWPset;

    currentWaypointNumber = 0;

    PathPlannerState initalPathPose = chosenWaypoints[0].pathPlannerSegment.getInitialState();

    this.swerve.resetOdometry(initalPathPose.poseMeters);

  }

  public void periodic() {
    waypointRunner(chosenWaypoints);
  }

  public void loadAutoPaths() {

    seg1 = PathPlanner.loadPath("4", 2.0, 1.5);
    seg2 = PathPlanner.loadPath("5", 2.0, 1.5);
    seg3 = PathPlanner.loadPath("6", 2.0, 1.5);
    seg4 = PathPlanner.loadPath("7", 2.0, 1.5);

    SquareAutoWPs = new Waypoint[]{
        // new Waypoint(new AutoSegmentedWaypoints()::moveUpperArm, 7.62, 0.75, seg1),
        // new Waypoint(new AutoSegmentedWaypoints()::moveLowerArm, 5.23, 1.97, seg2),
        // new Waypoint(new AutoSegmentedWaypoints()::moveBothArms, 0.74, 1.02, seg3)

        new Waypoint(this::moveUpperArm, seg1.getEndState().poseMeters.getX(), seg1.getEndState().poseMeters.getY(), seg1),
        new Waypoint(this::moveLowerArm, seg2.getEndState().poseMeters.getX(), seg2.getEndState().poseMeters.getY(), seg2),
        new Waypoint(this::moveBothArms, seg3.getEndState().poseMeters.getX(), seg3.getEndState().poseMeters.getY(), seg3),
        new Waypoint(this::moveUpperArm, seg4.getEndState().poseMeters.getX(), seg4.getEndState().poseMeters.getY(), seg4)

    };

    myAutoContainer = new AutoPose[]{
        new AutoPose("SquareAuto", 7.57, 1.84, -91.17, SquareAutoWPs),
    };
    for (AutoPose myAutoPose : myAutoContainer) {
      m_autoChooser.addOption(myAutoPose.name, myAutoPose);
    }
  }

  private void moveUpperArm() {

    double upperReference = 0.25;

    // Set the upper arm to go 45 degrees
    arm.setUpperArmReference(upperReference);
    arm.setLowerArmReference(0);


    if (SwerveTrajectory.trajectoryStatus.equals("done") &&

        (((upperReference + ArmConstants.UPPER_ARM_DEADBAND) <= arm.getUpperArmPosition() &&
            arm.getUpperArmPosition() <= (upperReference + ArmConstants.UPPER_ARM_DEADBAND)) ||

            (Timer.getFPGATimestamp() - autoDelay > 1.0))) {

      // Task to do when task is finished here:
      // arm.setUpperArmReference(0);

      if (chosenWaypoints.length != currentWaypointNumber + 1) {

        StateHasFinished = true;

      }

    } else if (!SwerveTrajectory.trajectoryStatus.equals("done")) {

      autoDelay = Timer.getFPGATimestamp();

    }
  }


  /**
   * This method moves the lower arm to a position
   * <p>
   *  If the lower arm is at the set position
   *  it will stop the auto path and move on to the next waypoint
   */
  private void moveLowerArm() {

    double lowerReference = 0.1;

    if (SwerveTrajectory.trajectoryStatus.equals("done")) {

      arm.setLowerArmReference(lowerReference);

    } else {

      autoDelay = Timer.getFPGATimestamp();

    }

    if (SwerveTrajectory.trajectoryStatus.equals("done") &&

        ((lowerReference - ArmConstants.LOWER_ARM_DEADBAND) <= arm.getLowerArmPosition() &&
            arm.getLowerArmPosition() <= (lowerReference + ArmConstants.LOWER_ARM_DEADBAND)) ||

        (Timer.getFPGATimestamp() - autoDelay > 1.0)) {

      arm.setLowerArmReference(0);

      if (chosenWaypoints.length != currentWaypointNumber + 1) {

        StateHasFinished = true;

      }
    }
  }

  /**
   * This method moves both arms to a position
   * <p>
   * If both arms are at the set position then
   * stop the auto path and move on to the next waypoint
   */
  private void moveBothArms() {

    // Where we want to put the arm
    // this is in revolutions
    double lowerArmReference = -0.1;
    double upperArmReference = 0;

    arm.setUpperArmReference(upperArmReference);

    if (SwerveTrajectory.trajectoryStatus.equals("done")) {

      arm.setLowerArmReference(lowerArmReference);

    } else {

      autoDelay = Timer.getFPGATimestamp();

    }

    // The if statement, in english:

    // If the trajectory is done,
    // and the lower and upper arms are near the desired positions (within the deadband)
    // and the task has started 1.5 seconds ago,
    // then set the arm references back to 0
    if (SwerveTrajectory.trajectoryStatus.equals("done") &&

        ((lowerArmReference - ArmConstants.LOWER_ARM_DEADBAND) <= arm.getLowerArmPosition() &&
            arm.getLowerArmPosition() <= (lowerArmReference + ArmConstants.LOWER_ARM_DEADBAND)) &&

        ((upperArmReference - ArmConstants.UPPER_ARM_DEADBAND) <= arm.getUpperArmPosition() &&
            arm.getUpperArmPosition() <= (upperArmReference + ArmConstants.UPPER_ARM_DEADBAND)) &&

        (Timer.getFPGATimestamp() - autoDelay > 1.5)) {

      arm.setLowerArmReference(0);
      arm.setUpperArmReference(0);

      if (chosenWaypoints.length != currentWaypointNumber + 1) {
        StateHasFinished = true;
      }
    }
  }

  private void moveBothArmsNoTimer() {

    arm.setUpperArmReference(-0.25);

    if (SwerveTrajectory.trajectoryStatus.equals("done")) {

      arm.setLowerArmReference(-0.1);

    } else {

      autoDelay = Timer.getFPGATimestamp();

    }

    if (SwerveTrajectory.trajectoryStatus.equals("done") &&
        (-0.23 <= arm.getLowerArmPosition() && arm.getLowerArmPosition() <= -0.27) &&
        (-0.23 <= arm.getUpperArmPosition() && arm.getUpperArmPosition() <= -0.27)) {
      System.out.println("done");

      arm.setLowerArmReference(0);
      arm.setUpperArmReference(0);

      if (chosenWaypoints.length != currentWaypointNumber + 1) {

        StateHasFinished = true;

      }
    }
  }

  public void done() {}

  public static class Waypoint {
    public Runnable action;
    public double posX;
    public double posY;
    public PathPlannerTrajectory pathPlannerSegment;

    public Waypoint(Runnable _action, double _x, double _y, PathPlannerTrajectory _PPS) {
      action = _action;
      posX = _x;
      posY = _y;
      pathPlannerSegment = _PPS;
    }
  }


  public static class AutoPose {

    public double thisX;
    public double thisY;
    public double thisRot;
    public Waypoint[] thisWPset;
    public String name;

    AutoPose(String _S, double _x, double _y, double _rot, Waypoint[] _WP) {
      thisX = _x;
      thisY = _y;
      thisRot = _rot;
      thisWPset = _WP;
      name = _S;
    }
  }

  public void waypointRunner(Waypoint[] thisWaypointSet) {
    // If we made one round with the state, we have successfully initialized
    if (!StateHasInitialized) {
      SwerveTrajectory.resetTrajectoryStatus();
      StateHasInitialized = true;
    }

    SwerveTrajectory.PathPlannerRunner(thisWaypointSet[currentWaypointNumber].pathPlannerSegment, swerve, swerve.getPose(), swerve.getPose().getRotation());

    thisWaypointSet[currentWaypointNumber].action.run();

    if (StateHasFinished) {
      currentWaypointNumber++;

      StateHasFinished = false;
      StateHasInitialized = false;
    }
  }
}