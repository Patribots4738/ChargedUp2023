package auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import math.Constants.AutoConstants;
import math.Constants.PlacementConstants;

public class AutoPathStorage implements Loggable {

  @Log(tabName = "AutoPicker", rowIndex = 0, columnIndex = 3, height = 1, width = 2)
  public static SendableChooser<AutoPose> autoChooser = new SendableChooser<>();

  public static Waypoint[] chosenWaypoints;

  public static AutoPose[] myAutoContainer;

  /*
   * Naming convention:
   * 1-9 represents the grid index starting from the bottom of the field in pathplanner
   * A-D represents the game piece index starting from the bottom of the field in pathplanner
   * CHARGE represents if the robot is going onto the charge pad
   * if CHARGE is excluded, the robot will not go onto the charge pad
   * H after 1-9 represents the high placement index for the arm
   * M after 1-9 represents the mid placement index for the arm
   * L after 1-9 represents the hybrid placement index for the arm
   */

  // Paths for the bottom of the field:
  // Start at grid index 1, place high, then go to field element A, intake, then go to grid index 2, and place high
  public static Waypoint[] _1H_A_2H_CHARGE;
  public static Waypoint[] _1H_A_2H;

  // Start at grid index 1, place high, then go to field element A, intake, then go to grid index 3, and place high
  public static Waypoint[] _1H_A_3H_CHARGE;
  public static Waypoint[] _1H_A_3H;

  // Start at grid index 2, place high, then go to field element A, intake, then go to grid index 1, and place high
  public static Waypoint[] _2H_A_1H_CHARGE;
  public static Waypoint[] _2H_A_1H;

  // Start at grid index 2, place high, then go to field element A, intake, then go to grid index 3, and place high
  public static Waypoint[] _2H_A_3H_CHARGE;
  public static Waypoint[] _2H_A_3H;

  // Start at grid index 3, place high, then go to field element A, intake, then go to grid index 1, and place high
  public static Waypoint[] _3H_A_1H_CHARGE;
  public static Waypoint[] _3H_A_1H;

  // Start at grid index 3, place high, then go to field element A, intake, then go to grid index 2, and place high
  public static Waypoint[] _3H_A_2H_CHARGE;
  public static Waypoint[] _3H_A_2H;


  // Paths for the top of the field:
  // Start at grid index 7, place high, then go to field element D, intake, then go to grid index 8, and place high
  public static Waypoint[] _7H_D_8H_CHARGE;
  public static Waypoint[] _7H_D_8H;

  // Start at grid index 7, place high, then go to field element D, intake, then go to grid index 9, and place high
  public static Waypoint[] _7H_D_9H_CHARGE;
  public static Waypoint[] _7H_D_9H;

  // Start at grid index 8, place high, then go to field element D, intake, then go to grid index 7, and place high
  public static Waypoint[] _8H_D_7H_CHARGE;
  public static Waypoint[] _8H_D_7H;

  // Start at grid index 8, place high, then go to field element D, intake, then go to grid index 9, and place high
  public static Waypoint[] _8H_D_9H_CHARGE;
  public static Waypoint[] _8H_D_9H;

  // Start at grid index 9, place high, then go to field element D, intake, then go to grid index 7, and place high
  public static Waypoint[] _9H_D_7H_CHARGE;
  public static Waypoint[] _9H_D_7H;

  // Start at grid index 9, place high, then go to field element D, intake, then go to grid index 8, and place high
  public static Waypoint[] _9H_D_8H_CHARGE;
  public static Waypoint[] _9H_D_8H;

  public static Waypoint[] _9H_D_7H_REACH;

  public static PathPlannerTrajectory square1;
  public static PathPlannerTrajectory square2;
  public static PathPlannerTrajectory square3;
  public static PathPlannerTrajectory square4;

  public static PathPlannerTrajectory _1;
  public static PathPlannerTrajectory _2;
  public static PathPlannerTrajectory _3;

  public static PathPlannerTrajectory _1_A;
  public static PathPlannerTrajectory _2_A;
  public static PathPlannerTrajectory _3_A;

  public static PathPlannerTrajectory _A_1;
  public static PathPlannerTrajectory _A_2;
  public static PathPlannerTrajectory _A_3;

  public static PathPlannerTrajectory _1_CH;
  public static PathPlannerTrajectory _2_CH;
  public static PathPlannerTrajectory _3_CH;


  public static PathPlannerTrajectory _7;
  public static PathPlannerTrajectory _8;
  public static PathPlannerTrajectory _9;

  public static PathPlannerTrajectory _7_D;
  public static PathPlannerTrajectory _8_D;
  public static PathPlannerTrajectory _9_D;

  public static PathPlannerTrajectory _D_7;
  public static PathPlannerTrajectory _D_8;
  public static PathPlannerTrajectory _D_9;

  public static PathPlannerTrajectory _7_CH;
  public static PathPlannerTrajectory _8_CH;
  public static PathPlannerTrajectory _9_CH;

  public static PathPlannerTrajectory _9_D_REACH;
  public static PathPlannerTrajectory _D_REACH_7;
  
  /**
   * Create all the waypoints needed for each path
   * then, after all has been completed, add them to the myAutoContainer array
   */
  public AutoPathStorage() {

    _1_A = PathPlanner.loadPath("_1_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_A = PathPlanner.loadPath("_2_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _3_A = PathPlanner.loadPath("_3_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _A_1 = PathPlanner.loadPath("_A_1", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _A_2 = PathPlanner.loadPath("_A_2", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _A_3 = PathPlanner.loadPath("_A_3", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    
    _1_CH = PathPlanner.loadPath("_1_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_CH = PathPlanner.loadPath("_2_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _3_CH = PathPlanner.loadPath("_3_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _7_D = PathPlanner.loadPath("_7_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_D = PathPlanner.loadPath("_8_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _9_D = PathPlanner.loadPath("_9_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _D_7 = PathPlanner.loadPath("_D_7", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_8 = PathPlanner.loadPath("_D_8", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_9 = PathPlanner.loadPath("_D_9", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _7_CH = PathPlanner.loadPath("_7_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_CH = PathPlanner.loadPath("_8_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _9_CH = PathPlanner.loadPath("_9_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _9_D_REACH = PathPlanner.loadPath("_9_D_REACH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_REACH_7 = PathPlanner.loadPath("_D_REACH_7", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // Use the initial state of _1_A as the starting point for _1
    // This is so we can move our arm before we move the robot.
    _1 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_1_A.getInitialState().poseMeters.getTranslation(), _1_A.getInitialState().poseMeters.getRotation(), _1_A.getInitialHolonomicPose().getRotation()),
        new PathPoint(_1_A.getInitialState().poseMeters.getTranslation(), _1_A.getInitialState().poseMeters.getRotation(), _1_A.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _2_A as the starting point for _2
    // This is so we can move our arm before we move the robot.
    _2 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_2_A.getInitialState().poseMeters.getTranslation(), _2_A.getInitialState().poseMeters.getRotation(), _2_A.getInitialHolonomicPose().getRotation()),
        new PathPoint(_2_A.getInitialState().poseMeters.getTranslation(), _2_A.getInitialState().poseMeters.getRotation(), _2_A.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _3_A as the starting point for _3
    // This is so we can move our arm before we move the robot.
    _3 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_3_A.getInitialState().poseMeters.getTranslation(), _3_A.getInitialState().poseMeters.getRotation(), _3_A.getInitialHolonomicPose().getRotation()),
        new PathPoint(_3_A.getInitialState().poseMeters.getTranslation(), _3_A.getInitialState().poseMeters.getRotation(), _3_A.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _7_D as the starting point for _7
    // This is so we can move our arm before we move the robot.
    _7 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_7_D.getInitialState().poseMeters.getTranslation(), _7_D.getInitialState().poseMeters.getRotation(), _7_D.getInitialHolonomicPose().getRotation()),
        new PathPoint(_7_D.getInitialState().poseMeters.getTranslation(), _7_D.getInitialState().poseMeters.getRotation(), _7_D.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _8_D as the starting point for _8
    // This is so we can move our arm before we move the robot.
    _8 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_8_D.getInitialState().poseMeters.getTranslation(), _8_D.getInitialState().poseMeters.getRotation(), _8_D.getInitialHolonomicPose().getRotation()),
        new PathPoint(_8_D.getInitialState().poseMeters.getTranslation(), _8_D.getInitialState().poseMeters.getRotation(), _8_D.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _9_D as the starting point for _9
    // This is so we can move our arm before we move the robot.
    _9 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_9_D.getInitialState().poseMeters.getTranslation(), _9_D.getInitialState().poseMeters.getRotation(), _9_D.getInitialHolonomicPose().getRotation()),
        new PathPoint(_9_D.getInitialState().poseMeters.getTranslation(), _9_D.getInitialState().poseMeters.getRotation(), _9_D.getInitialHolonomicPose().getRotation())
    );


    /**
     * The syntax here is as follows:
     * PATH NAME = new Waypoint[] {
     *  new Waypoint(
     *    PATH,
     *    ARM_PLACEMENT INDEX,
     *    CLAW_SPEED
     *  ),
     *  new Waypoint(
     *    PATH_2,
     *    ARM_PLACEMENT INDEX,
     *    CLAW_SPEED
     *  )
     *  ...
     * }
     * 
     * If the path is 1/3/7/9, then the arm will go to a cone index
     * If the path is 2/4/5/8, then the arm will go to a cube index
     * When going to a number, the claw will outtake, and the arm will go to a placement position
     * H = high, M = mid, L = hybrid
     * When going to a letter, the claw will intake, and the arm will go to the floor intake
     * When going to a charge, the claw will stop, and arm stowed
     * 
     * Create waypoint arrays in order as defined from the top of the file
     */

    // WPs for the bottom of the field:
    _1H_A_2H_CHARGE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_1_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CUBE_INTAKE_SPEED),
        new Waypoint(_A_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_2_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _1H_A_2H = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_1_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CUBE_INTAKE_SPEED),
        new Waypoint(_A_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _1H_A_3H_CHARGE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_1_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_3_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _1H_A_3H = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_1_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _2H_A_1H_CHARGE = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_2_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_1_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _2H_A_1H = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_2_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _2H_A_3H_CHARGE = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_2_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_3_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _2H_A_3H = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_2_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _3H_A_1H_CHARGE = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_3_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_1_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _3H_A_1H = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_3_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_1, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _3H_A_2H_CHARGE = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_3_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_2_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _3H_A_2H = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_3_A, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_A_2, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    // WPs for the top of the field:
    _7H_D_8H_CHARGE = new Waypoint[] {
      new Waypoint(_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
      new Waypoint(_7_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CUBE_INTAKE_SPEED),
      new Waypoint(_D_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
      new Waypoint(_8_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _7H_D_8H = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_7_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CUBE_INTAKE_SPEED),
        new Waypoint(_D_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _7H_D_9H_CHARGE = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_7_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _7H_D_9H = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_7_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _8H_D_7H_CHARGE = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_8_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_7_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _8H_D_7H = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_8_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _8H_D_9H_CHARGE = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_8_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _8H_D_9H = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_8_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _9H_D_7H_CHARGE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_7_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _9H_D_7H = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _9H_D_8H_CHARGE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CUBE_INTAKE_SPEED),
        new Waypoint(_D_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_8_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _9H_D_8H = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_D, PlacementConstants.FLOOR_INTAKE_INDEX, PlacementConstants.CLAW_CUBE_INTAKE_SPEED),
        new Waypoint(_D_8, PlacementConstants.HIGH_CUBE_LAUNCH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
    };

    _9H_D_7H_REACH = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_9_D_REACH, PlacementConstants.LONG_ARM_REACH_INDEX, PlacementConstants.CLAW_CONE_INTAKE_SPEED),
        new Waypoint(_D_REACH_7, PlacementConstants.HIGH_CONE_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED),
        new Waypoint(_7_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    myAutoContainer = new AutoPose[] {
      // AutoPoses for the bottom of the field:
      new AutoPose("1H_A_2H_CHARGE", _1H_A_2H_CHARGE),
      new AutoPose("1H_A_2H", _1H_A_2H),
      new AutoPose("1H_A_3H_CHARGE", _1H_A_3H_CHARGE),
      new AutoPose("1H_A_3H", _1H_A_3H),
      new AutoPose("2H_A_1H_CHARGE", _2H_A_1H_CHARGE),
      new AutoPose("2H_A_1H", _2H_A_1H),
      new AutoPose("2H_A_3H_CHARGE", _2H_A_3H_CHARGE),
      new AutoPose("2H_A_3H", _2H_A_3H),
      new AutoPose("3H_A_1H_CHARGE", _3H_A_1H_CHARGE),
      new AutoPose("3H_A_1H", _3H_A_1H),
      new AutoPose("3H_A_2H_CHARGE", _3H_A_2H_CHARGE),
      new AutoPose("3H_A_2H", _3H_A_2H),
      // AutoPoses for the top of the field:
      new AutoPose("7H_D_8H_CHARGE", _7H_D_8H_CHARGE),
      new AutoPose("7H_D_8H", _7H_D_8H),
      new AutoPose("7H_D_9H_CHARGE", _7H_D_9H_CHARGE),
      new AutoPose("7H_D_9H", _7H_D_9H),
      new AutoPose("8H_D_7H_CHARGE", _8H_D_7H_CHARGE),
      new AutoPose("8H_D_7H", _8H_D_7H),
      new AutoPose("8H_D_9H_CHARGE", _8H_D_9H_CHARGE),
      new AutoPose("8H_D_9H", _8H_D_9H),
      new AutoPose("9H_D_7H_CHARGE", _9H_D_7H_CHARGE),
      new AutoPose("9H_D_7H", _9H_D_7H),
      new AutoPose("9H_D_8H_CHARGE", _9H_D_8H_CHARGE),
      new AutoPose("9H_D_8H", _9H_D_8H),
      new AutoPose("9H_D_7H_REACH", _9H_D_7H_REACH)
    };

    for (AutoPose AutoPose : myAutoContainer) {
      autoChooser.addOption(AutoPose.name, AutoPose);
    }
  }

  // The waypoint class is used to create a waypoint set
  // This is so we can create a larget path using multiple waypoints
  public static class Waypoint {

    public PathPlannerTrajectory pathPlannerSegment;
    public int armPosIndex;
    public double clawDirection;

    public Waypoint(PathPlannerTrajectory _PPS, int _index, double _clawDirection) {
      armPosIndex = _index;
      clawDirection = _clawDirection;
      pathPlannerSegment = _PPS;
    }
  }

  // The auto pose class is used to create a waypoint set with a string
  // This is so we can have it in a drop down menu in shuffleboard
  @SuppressWarnings({"CanBeFinal", "SameParameterValue"})
  public static class AutoPose {
    
    public String name;
    public Waypoint[] thisWPset;

    AutoPose(String _S, Waypoint[] _WP) {
      thisWPset = _WP;
      name = _S;
    }
  }
}
