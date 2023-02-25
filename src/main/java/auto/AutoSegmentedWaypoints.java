// Referenced from https://github.com/Stampede3630/2022-Code/blob/0ad2aa434f50d8f5dc93e965809255f697dadffe/src/main/java/frc/robot/AutoSegmentedWaypoints.java#L81
package auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import hardware.Arm;
import hardware.Claw;
import hardware.Swerve;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import math.Constants.PlacementConstants;

public class AutoSegmentedWaypoints implements Loggable {

  Swerve swerve;
  Arm arm;
  Claw claw;

  public Waypoint[] HighFiveBallAutoWPs;
  public Waypoint[] SquareAutoWPs;
  public Waypoint[] ConeToCubeWPs;

  public Waypoint[] chosenWaypoints;

  public int currentWaypointNumber = 0;

  public AutoPose chosenPath;
  public AutoPose[] myAutoContainer;

  public PathPlannerTrajectory square1;
  public PathPlannerTrajectory square2;
  public PathPlannerTrajectory square3;
  public PathPlannerTrajectory square4;

  public PathPlannerTrajectory coneToCube0;
  public PathPlannerTrajectory coneToCube1;
  public PathPlannerTrajectory coneToCube2;
  public PathPlannerTrajectory coneToCube3;

  @Log
  public double autoDelay;

  public boolean stateHasFinished = false;
  public boolean stateHasInitialized = false;
  public boolean clawHasStarted = false;

  @Log(tabName = "CompetitionLogger", rowIndex = 0, columnIndex = 3, height = 1, width = 2)
  public SendableChooser<AutoPose> m_autoChooser = new SendableChooser<>();


  public AutoSegmentedWaypoints(Swerve swerve, Arm arm, Claw claw) {
    this.swerve = swerve;
    this.arm = arm;
    this.claw = claw;
  }

  public void init() {

    if (m_autoChooser.getSelected() == null) {

      chosenPath = myAutoContainer[0];

    } else {

      chosenPath = m_autoChooser.getSelected();

    }

    chosenWaypoints = chosenPath.thisWPset;

    currentWaypointNumber = 0;

    PathPlannerState initialPathPose = chosenWaypoints[0].pathPlannerSegment.getInitialState();

    swerve.resetOdometry(new Pose2d(initialPathPose.poseMeters.getTranslation(), initialPathPose.holonomicRotation));

  }

  public void periodic() {
    waypointRunner(chosenWaypoints);
  }

  public void loadAutoPaths() {

    square1 = PathPlanner.loadPath("Square1", 4.0, 3.0);
    square2 = PathPlanner.loadPath("Square2", 4.0, 3.0);
    square3 = PathPlanner.loadPath("Square3", 2.0, 1.5);
    square4 = PathPlanner.loadPath("Square", 2.0, 1.5);

    coneToCube1 = PathPlanner.loadPath("ConeToCube1", 2.0, 1.5);
    coneToCube2 = PathPlanner.loadPath("ConeToCube2", 2.0, 1.5);
    coneToCube3 = PathPlanner.loadPath("ConeToCube3", 2.0, 1.5);
    // create a new path just using the initial state of coneToCube1
    coneToCube0 =  PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),

        new PathPoint(coneToCube1.getInitialState().poseMeters.getTranslation(),
            coneToCube1.getInitialState().poseMeters.getRotation(),
            coneToCube1.getInitialState().poseMeters.getRotation()),

        new PathPoint(coneToCube1.getInitialState().poseMeters.getTranslation(),
            coneToCube1.getInitialState().poseMeters.getRotation(),
            coneToCube1.getInitialState().poseMeters.getRotation())
        
    );
    
    SquareAutoWPs = new Waypoint[]{
      new Waypoint(
          square1,
          PlacementConstants.STOWED_PLACEMENT_INDEX,
          PlacementConstants.CLAW_STOPPED_SPEED
      ),
      new Waypoint(
          square2,
          PlacementConstants.STOWED_PLACEMENT_INDEX,
          PlacementConstants.CLAW_STOPPED_SPEED
      )
      // ,
      // new Waypoint(
      //         PlacementConstants.HIGH_CONE_PLACEMENT_INDEX,
      //         PlacementConstants.CLAW_INTAKE_SPEED,
      //         square3
      // ),
      // new Waypoint(
      //         PlacementConstants.STOWED_PLACEMENT_INDEX,
      //         PlacementConstants.CLAW_STOPPED_SPEED,
      //         square4
      // )
    };

    ConeToCubeWPs = new Waypoint[] {
      new Waypoint(
          coneToCube0,
          PlacementConstants.HIGH_CONE_PLACEMENT_INDEX,
          PlacementConstants.CLAW_OUTTAKE_SPEED
      ),
      new Waypoint(
          coneToCube1,
          PlacementConstants.FLOOR_INTAKE_PLACEMENT_INDEX,
          PlacementConstants.CLAW_CUBE_INTAKE_SPEED
      ),
      new Waypoint(
          coneToCube2,
          PlacementConstants.HIGH_CUBE_LAUNCH_INDEX,
          PlacementConstants.CLAW_OUTTAKE_SPEED
      ),
      new Waypoint(
          coneToCube3,
          PlacementConstants.STOWED_PLACEMENT_INDEX,
          PlacementConstants.CLAW_STOPPED_SPEED
      )
    };

    myAutoContainer = new AutoPose[]{
      new AutoPose("SquareAuto", square1.getInitialState().poseMeters.getX(),
              square1.getInitialState().poseMeters.getY(),
              square1.getInitialState().poseMeters.getRotation().getDegrees(),
              SquareAutoWPs
      ),
      new AutoPose("ConeToCube", coneToCube0.getInitialState().poseMeters.getX(),
              coneToCube0.getInitialState().poseMeters.getY(),
              coneToCube0.getInitialState().poseMeters.getRotation().getDegrees(),
              ConeToCubeWPs
      )
    };

    for (AutoPose myAutoPose : myAutoContainer) {
      m_autoChooser.addOption(myAutoPose.name, myAutoPose);
    }
  }

  /**
   * This method moves both arms to a position
   * <p>
   * If both arms are in the set position, then
   * stop the auto path and move on to the next waypoint
   */
  private void setArmIndex(int armIndex, double clawSpeed) {

    // // Only move the claw before the arm
    // // if it needs to hold a game piece
    if (claw.getDesiredSpeed() != PlacementConstants.CLAW_CONE_INTAKE_SPEED && 
        claw.getDesiredSpeed() != PlacementConstants.CLAW_CUBE_INTAKE_SPEED) {
      claw.setDesiredSpeed(PlacementConstants.CLAW_STOPPED_SPEED);
    }

    if (SwerveTrajectory.trajectoryStatus.equals("done")) {

      arm.setArmIndex(armIndex);

    } else {

      autoDelay = Timer.getFPGATimestamp();

    }
    
    if (SwerveTrajectory.trajectoryStatus.equals("done") && arm.getAtDesiredPositions()) {

      if (!clawHasStarted) { 
        autoDelay = Timer.getFPGATimestamp();
        clawHasStarted = true;
      }
      claw.setDesiredSpeed(clawSpeed);
      // 1.5 seconds since the path has completed
      if (Timer.getFPGATimestamp() - autoDelay > 1) {
        
        if (currentWaypointNumber < chosenWaypoints.length - 1) {
          stateHasFinished = true;
        }

      }
    }
  }

  public static class Waypoint {
    public int armPosIndex;
    public double clawDirection;
    public PathPlannerTrajectory pathPlannerSegment;

    public Waypoint(PathPlannerTrajectory _PPS, int _index, double _clawDirection) {
      armPosIndex = _index;
      clawDirection = _clawDirection;
      pathPlannerSegment = _PPS;
    }
  }


  @SuppressWarnings({"CanBeFinal", "SameParameterValue"})
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
    if (!stateHasInitialized) {
      SwerveTrajectory.resetTrajectoryStatus();
      stateHasInitialized = true;
    }

    SwerveTrajectory.PathPlannerRunner(thisWaypointSet[currentWaypointNumber].pathPlannerSegment, swerve);

    this.setArmIndex(thisWaypointSet[currentWaypointNumber].armPosIndex, thisWaypointSet[currentWaypointNumber].clawDirection);

    if (stateHasFinished) {

      arm.setArmIndex(PlacementConstants.STOWED_PLACEMENT_INDEX);
      
      currentWaypointNumber++;

      stateHasFinished = false;
      stateHasInitialized = false;
      clawHasStarted = false;
    }
  }
}