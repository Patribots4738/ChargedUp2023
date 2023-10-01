// Be warned, dragons ahead!
// For teams in the future trying to create this kind of complexity in autonomous
// Do not do it this way! 
// Instead, use a hashmap that has a key of the trajectory and the value of the arm and claw movement for the key
// T
package auto;

import calc.Constants.AutoConstants;
import calc.Constants.PlacementConstants;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import frc.robot.DriverUI;
import io.github.oblarg.oblog.Loggable;

public class AutoPathStorage implements Loggable {

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

  public static Waypoint[] _MOBILITY_ONLY;

  public static Waypoint[] _SQUARE_HALF;

  // Paths for the bottom of the field:
  // Start at grid index 1, place high, then go to field element A, intake, then go to grid index 2, and place high
  public static Waypoint[] _1H_A_2H_CHARGE_BLUE;
  public static Waypoint[] _1H_A_2H_CHARGE_RED;
  public static Waypoint[] _1H_A_2H;
  public static Waypoint[] _1H_A_2H_B;

  // Start at grid index 1, place high, then go to field element A, intake, then go to grid index 3, and place high
  public static Waypoint[] _1H_A;
  public static Waypoint[] _1H_A_3H_CHARGE;
  public static Waypoint[] _1H_A_3H;
  public static Waypoint[] _1H_A_3H_B;  

  // Start at grid index 2, place high, then go to field element A, intake, then go to grid index 1, and place high
  public static Waypoint[] _2H_A_1H_CHARGE;
  public static Waypoint[] _2H_A_1H;
  public static Waypoint[] _2H_A;
  public static Waypoint[] _2H_A_1H_B;

  // Start at grid index 2, place high, then go to field element A, intake, then go to grid index 3, and place high
  public static Waypoint[] _2H_A_3H_CHARGE;
  public static Waypoint[] _2H_A_3H;
  public static Waypoint[] _2H_A_3H_B;

  // Start at grid index 3, place high, then go to field element A, intake, then go to grid index 1, and place high
  public static Waypoint[] _3H_A_1H_CHARGE;
  public static Waypoint[] _3H_A_1H;
  public static Waypoint[] _3H_A;
  public static Waypoint[] _3H_A_1H_B;

  // Start at grid index 3, place high, then go to field element A, intake, then go to grid index 2, and place high
  public static Waypoint[] _3H_A_2H_CHARGE;
  public static Waypoint[] _3H_A_2H;
  public static Waypoint[] _3H_A_2H_B;


  // Paths for the top of the field:
  // Start at grid index 7, place high, then go to field element D, intake, then go to grid index 8, and place high
  public static Waypoint[] _7H_D_8H_CHARGE;
  public static Waypoint[] _7H_D_8H;
  public static Waypoint[] _7H_D;
  public static Waypoint[] _7H_D_8H_C;

  // Start at grid index 7, place high, then go to field element D, intake, then go to grid index 9, and place high
  public static Waypoint[] _7H_D_9H_CHARGE;
  public static Waypoint[] _7H_D_9H;
  public static Waypoint[] _7H_D_9H_C;

  // Start at grid index 8, place high, then go to field element D, intake, then go to grid index 7, and place high
  public static Waypoint[] _8H_D_7H_CHARGE;
  public static Waypoint[] _8H_D_7H;
  public static Waypoint[] _8H_D;
  public static Waypoint[] _8H_D_7H_C;

  // Start at grid index 8, place high, then go to field element D, intake, then go to grid index 9, and place high
  public static Waypoint[] _8H_D_9H_CHARGE;
  public static Waypoint[] _8H_D_9H;
  public static Waypoint[] _8H_D_9H_C;

  // Start at grid index 9, place high, then go to field element D, intake, then go to grid index 7, and place high
  public static Waypoint[] _9H_D_7H_CHARGE;
  public static Waypoint[] _9H_D_7H;
  public static Waypoint[] _9H_D;
  public static Waypoint[] _9H_D_7H_C;

  // Start at grid index 9, place high, then go to field element D, intake, then go to grid index 8, and place high
  public static Waypoint[] _9H_D_8H_CHARGE_BLUE;
  public static Waypoint[] _9H_D_8H_CHARGE_RED;
  public static Waypoint[] _9H_D_8H;
  public static Waypoint[] _9H_D_8H_C;

  public static Waypoint[] _9H_D_8H_C_8M_RED;
  public static Waypoint[] _9H_D_8H_C_CHARGE_RED;

  public static Waypoint[] _9H_D_8H_C_8M_BLUE;
  public static Waypoint[] _9H_D_8H_C_CHARGE_BLUE;
  
  public static Waypoint[] _1H_A_2H_B_2M_RED;
  public static Waypoint[] _1H_A_2H_B_CHARGE_RED;

  public static Waypoint[] _1H_A_2H_B_2M_BLUE;
  public static Waypoint[] _1H_A_2H_B_CHARGE_BLUE;

  public static Waypoint[] _4H_MOBILITY;
  public static Waypoint[] _5H_MOBILITY;
  public static Waypoint[] _6H_MOBILITY;
  public static Waypoint[] _6H_ONLY;
  public static Waypoint[] _4H_MOBILITY_CHARGE;
  public static Waypoint[] _5H_MOBILITY_CHARGE;
  public static Waypoint[] _6H_MOBILITY_CHARGE;

  public static Waypoint[] _1H_A_CHARGE;
  public static Waypoint[] _2H_A_CHARGE;
  public static Waypoint[] _3H_A_CHARGE;
  public static Waypoint[] _7H_D_CHARGE;
  public static Waypoint[] _8H_D_CHARGE;
  public static Waypoint[] _9H_D_CHARGE;

  public static Waypoint[] _9H_D_7H_REACH;
  public static Waypoint[] _MOBILITY_CHARGE_TEST_ONLY;

  public static Waypoint[] _POOF_1H_A_2H_B_2M_BLUE;
  public static Waypoint[] _POOF_1H_A_2H_B_2M_RED;
  public static Waypoint[] _POOF_9H_D_8H_C_8M;

  public static Waypoint[] _SPIN;

  public static PathPlannerTrajectory square1;
  public static PathPlannerTrajectory square2;
  public static PathPlannerTrajectory square3;
  public static PathPlannerTrajectory square4;

  public static PathPlannerTrajectory _MOBILITY;

  public static PathPlannerTrajectory _1;
  public static PathPlannerTrajectory _2;
  public static PathPlannerTrajectory _3;
  public static PathPlannerTrajectory _4;
  public static PathPlannerTrajectory _5;
  public static PathPlannerTrajectory _6;
  public static PathPlannerTrajectory _7;
  public static PathPlannerTrajectory _8;
  public static PathPlannerTrajectory _9;

  public static PathPlannerTrajectory _1_A;
  public static PathPlannerTrajectory _1_B;
  public static PathPlannerTrajectory _2_A;
  public static PathPlannerTrajectory _2_B;
  public static PathPlannerTrajectory _3_A;
  public static PathPlannerTrajectory _3_B;
  public static PathPlannerTrajectory _4_B;
  public static PathPlannerTrajectory _5_B;
  public static PathPlannerTrajectory _5_C;
  public static PathPlannerTrajectory _6_C;

  public static PathPlannerTrajectory _A_1;
  public static PathPlannerTrajectory _A_2;
  public static PathPlannerTrajectory _A_3;
  public static PathPlannerTrajectory _A_4;
  public static PathPlannerTrajectory _A_M;

  public static PathPlannerTrajectory _B_4;
  public static PathPlannerTrajectory _B_5;

  public static PathPlannerTrajectory _C_5;
  public static PathPlannerTrajectory _C_6;

  public static PathPlannerTrajectory _1_CH;
  public static PathPlannerTrajectory _2_CH;
  public static PathPlannerTrajectory _3_CH;
  public static PathPlannerTrajectory _4_CH;
  public static PathPlannerTrajectory _5_CH;
  public static PathPlannerTrajectory _6_CH;
  public static PathPlannerTrajectory _7_CH;
  public static PathPlannerTrajectory _8_CH;
  public static PathPlannerTrajectory _9_CH;

  public static PathPlannerTrajectory _4_M_CH;
  public static PathPlannerTrajectory _5_M;
  public static PathPlannerTrajectory _6_M_CH;
  public static PathPlannerTrajectory _M_CH;

  public static PathPlannerTrajectory _7_C;
  public static PathPlannerTrajectory _7_D;
  public static PathPlannerTrajectory _8_C;
  public static PathPlannerTrajectory _8_D;
  public static PathPlannerTrajectory _9_C;
  public static PathPlannerTrajectory _9_D;
  public static PathPlannerTrajectory _9_D_8_RED;
  public static PathPlannerTrajectory _9_D_8_BLUE;
  public static PathPlannerTrajectory _8_C_8_RED;
  public static PathPlannerTrajectory _8_C_8_BLUE;
  public static PathPlannerTrajectory _8_C_CH_RED;
  public static PathPlannerTrajectory _8_C_CH_BLUE;

  public static PathPlannerTrajectory _POOF_1_A_2;
  public static PathPlannerTrajectory _POOF_1_A_2_RED;
  public static PathPlannerTrajectory _POOF_2_B_2;
  public static PathPlannerTrajectory _POOF_2_B_2_RED;
  public static PathPlannerTrajectory _POOF_9_D_8;
  public static PathPlannerTrajectory _POOF_8_C_8;

  public static PathPlannerTrajectory _1_A_2_BLUE;
  public static PathPlannerTrajectory _1_A_2_RED;
  public static PathPlannerTrajectory _2_B_2_BLUE;
  public static PathPlannerTrajectory _2_B_2_RED;

  public static PathPlannerTrajectory _2_B_CH_BLUE;
  public static PathPlannerTrajectory _2_B_CH_RED;

  public static PathPlannerTrajectory _D_6;
  public static PathPlannerTrajectory _D_7;
  public static PathPlannerTrajectory _D_8;
  public static PathPlannerTrajectory _D_9;
  public static PathPlannerTrajectory _D_M;

  public static PathPlannerTrajectory _9_D_REACH;
  public static PathPlannerTrajectory _D_REACH_7;
  public static PathPlannerTrajectory _CCWSPIN;
  
  /**
   * Create all the waypoints needed for each path
   * then, after all has been completed, add them to the myAutoContainer array
   */
  public AutoPathStorage() {

    square1 = PathPlanner.loadPath("Square1", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    square2 = PathPlanner.loadPath("Square2", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    square3 = PathPlanner.loadPath("Square3", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    square4 = PathPlanner.loadPath("Square4", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _MOBILITY = PathPlanner.loadPath("_MOBILITY", AutoConstants.MAX_SPEED_METERS_PER_SECOND/2.0, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED/2.0);

    _1_A = PathPlanner.loadPath("_1_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _1_B = PathPlanner.loadPath("_1_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_B = PathPlanner.loadPath("_2_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_A = PathPlanner.loadPath("_2_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _3_A = PathPlanner.loadPath("_3_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _3_B = PathPlanner.loadPath("_3_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _4_B = PathPlanner.loadPath("_4_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _4_M_CH = PathPlanner.loadPath("_4_M", 2, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _5_B = PathPlanner.loadPath("_5_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _5_C = PathPlanner.loadPath("_5_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _5_M = PathPlanner.loadPath("_5_M", 2, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _6_C = PathPlanner.loadPath("_6_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _6_M_CH = PathPlanner.loadPath("_6_M", 2, 1);
    _7_C = PathPlanner.loadPath("_7_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _7_D = PathPlanner.loadPath("_7_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_C = PathPlanner.loadPath("_8_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_D = PathPlanner.loadPath("_8_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _9_C = PathPlanner.loadPath("_9_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _9_D = PathPlanner.loadPath("_9_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _A_1 = PathPlanner.loadPath("_A_1", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _A_2 = PathPlanner.loadPath("_A_2", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _A_3 = PathPlanner.loadPath("_A_3", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _A_4 = PathPlanner.loadPath("_A_4", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _A_M = PathPlanner.loadPath("_A_M", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _B_4 = PathPlanner.loadPath("_B_4", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _B_5 = PathPlanner.loadPath("_B_5", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _C_5 = PathPlanner.loadPath("_C_5", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _C_6 = PathPlanner.loadPath("_C_6", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _D_6 = PathPlanner.loadPath("_D_6", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_7 = PathPlanner.loadPath("_D_7", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_8 = PathPlanner.loadPath("_D_8", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_9 = PathPlanner.loadPath("_D_9", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _D_M = PathPlanner.loadPath("_D_M", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _1_CH = PathPlanner.loadPath("_1_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_CH = PathPlanner.loadPath("_2_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _3_CH = PathPlanner.loadPath("_3_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _4_CH = PathPlanner.loadPath("_4_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _5_CH = PathPlanner.loadPath("_5_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _6_CH = PathPlanner.loadPath("_6_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _7_CH = PathPlanner.loadPath("_7_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_CH = PathPlanner.loadPath("_8_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, 3);
    _9_CH = PathPlanner.loadPath("_9_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _M_CH = PathPlanner.loadPath("_M_CH", 3, 1);

    _9_D_REACH = PathPlanner.loadPath("_9_D_REACH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _9_D_8_RED = PathPlanner.loadPath("9_D_8_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _9_D_8_BLUE = PathPlanner.loadPath("9_D_8_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_C_8_RED = PathPlanner.loadPath("8_C_8_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_C_CH_RED = PathPlanner.loadPath("8_C_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _1_A_2_BLUE = PathPlanner.loadPath("1_A_2_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _1_A_2_RED = PathPlanner.loadPath("1_A_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_B_2_BLUE = PathPlanner.loadPath("2_B_2_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_B_2_RED = PathPlanner.loadPath("2_B_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _8_C_8_BLUE = PathPlanner.loadPath("8_C_8_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_C_CH_BLUE = PathPlanner.loadPath("8_C_CH_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _8_C_CH_RED = PathPlanner.loadPath("8_C_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_B_CH_RED = PathPlanner.loadPath("2_B_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _2_B_CH_BLUE = PathPlanner.loadPath("2_B_CH_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _D_REACH_7 = PathPlanner.loadPath("_D_REACH_7", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    _POOF_1_A_2 = PathPlanner.loadPath("_POOF_1_A_2", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _POOF_2_B_2 = PathPlanner.loadPath("_POOF_2_B_2", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _POOF_1_A_2_RED = PathPlanner.loadPath("_POOF_1_A_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _POOF_2_B_2_RED = PathPlanner.loadPath("_POOF_2_B_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    

    _POOF_9_D_8 = PathPlanner.loadPath("_POOF_9_D_8", 2.75, 3.3);
    _POOF_8_C_8 = PathPlanner.loadPath("_POOF_8_C_8", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    
    _CCWSPIN = PathPlanner.loadPath("SPIN", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // Use the initial state of _1_A as the starting point for _1
    // This is so we can move our arm before we move the robot.
    _1 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_1_A_2_BLUE.getInitialState().poseMeters.getTranslation(), _1_A_2_BLUE.getInitialState().poseMeters.getRotation(), _1_A_2_BLUE.getInitialHolonomicPose().getRotation()),
        new PathPoint(_1_A_2_BLUE.getInitialState().poseMeters.getTranslation(), _1_A_2_BLUE.getInitialState().poseMeters.getRotation(), _1_A_2_BLUE.getInitialHolonomicPose().getRotation())
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

    // Use the initial state of _4_B as the starting point for _4
    // This is so we can move our arm before we move the robot.
    _4 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_4_B.getInitialState().poseMeters.getTranslation(), _4_B.getInitialState().poseMeters.getRotation(), _4_B.getInitialHolonomicPose().getRotation()),
        new PathPoint(_4_B.getInitialState().poseMeters.getTranslation(), _4_B.getInitialState().poseMeters.getRotation(), _4_B.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _5_B as the starting point for _5
    // This is so we can move our arm before we move the robot.
    _5 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_5_B.getInitialState().poseMeters.getTranslation(), _5_B.getInitialState().poseMeters.getRotation(), _5_B.getInitialHolonomicPose().getRotation()),
        new PathPoint(_5_B.getInitialState().poseMeters.getTranslation(), _5_B.getInitialState().poseMeters.getRotation(), _5_B.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _6_C as the starting point for _6
    // This is so we can move our arm before we move the robot.
    _6 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_6_C.getInitialState().poseMeters.getTranslation(), _6_C.getInitialState().poseMeters.getRotation(), _6_C.getInitialHolonomicPose().getRotation()),
        new PathPoint(_6_C.getInitialState().poseMeters.getTranslation(), _6_C.getInitialState().poseMeters.getRotation(), _6_C.getInitialHolonomicPose().getRotation())
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
        new PathPoint(_9_D_8_RED.getInitialState().poseMeters.getTranslation(), _9_D_8_RED.getInitialState().poseMeters.getRotation(), _9_D_8_RED.getInitialHolonomicPose().getRotation()),
        new PathPoint(_9_D_8_RED.getInitialState().poseMeters.getTranslation(), _9_D_8_RED.getInitialState().poseMeters.getRotation(), _9_D_8_RED.getInitialHolonomicPose().getRotation())
    );


    /*
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
     * If the path is 1/3/7/9, then the arm will go to a cone index
     * If the path is 2/4/5/8, then the arm will go to a cube index
     * When going to a number, the claw will outtake, and the arm will go to a placement position
     * H = high, M = mid, L = hybrid
     * When going to a letter, the claw will intake, and the arm will go to the floor intake
     * When going to a charge, the claw will  stop, and arm stowed
     * Create waypoint arrays in order as defined from the top of the file
     */

    _SQUARE_HALF = new Waypoint[] {
        new Waypoint(square1, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(square2, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _MOBILITY_ONLY = new Waypoint[] {
        new Waypoint(_MOBILITY, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    // WPs for the bottom of the field:
    _1H_A_2H_CHARGE_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_RED, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_2_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _1H_A_2H_CHARGE_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_2_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _1H_A_2H = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_A_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
    };

    _1H_A = new Waypoint[] {
      new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
      new Waypoint(_1_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE)
    };

    _1H_A_2H_B = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_A_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_B, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _1H_A_3H_CHARGE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _1H_A_3H = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
    };

    _1H_A_3H_B = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_B, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _2H_A_1H_CHARGE = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _2H_A_1H = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
    };

    _2H_A = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };
    
    _2H_A_1H_B = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_B, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _2H_A_3H_CHARGE = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _2H_A_3H = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
    };

    _2H_A_3H_B = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_B, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _3H_A_1H_CHARGE = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _3H_A_1H = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
    };

    _3H_A = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };
    
    _3H_A_1H_B = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_A_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_B, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _3H_A_2H_CHARGE = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_A_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _3H_A_2H = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_A_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE)
    };

    _3H_A_2H_B = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_A_2, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_B, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    // WPs for the top of the field:
    _7H_D_8H_CHARGE = new Waypoint[] {
      new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
      new Waypoint(_7_D, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
      new Waypoint(_D_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
      new Waypoint(_8_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _7H_D_8H = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_D_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE)
    };

    _7H_D = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE)
    };
    
    _7H_D_8H_C = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
        new Waypoint(_D_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_C, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _7H_D_9H_CHARGE = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _7H_D_9H = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE)
    };

    _7H_D_9H_C = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_C, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _8H_D_7H_CHARGE = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _8H_D_7H = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE)
    };

    _8H_D = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };
    
    _8H_D_7H_C = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_C, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _8H_D_9H_CHARGE = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _8H_D_9H = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE)
    };

    _8H_D_9H_C = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_C, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _9H_D_7H_CHARGE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _9H_D_7H = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE)
    };

    _9H_D = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };
    
    _9H_D_7H_C = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_C, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _4H_MOBILITY = new Waypoint[] {
        new Waypoint(_4, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_4_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _5H_MOBILITY = new Waypoint[] {
        new Waypoint(_5, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_5_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _6H_ONLY = new Waypoint[] {
        new Waypoint(_6, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_6, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _6H_MOBILITY = new Waypoint[] {
        new Waypoint(_6, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_6_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _4H_MOBILITY_CHARGE = new Waypoint[] {
        new Waypoint(_4, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_4_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _5H_MOBILITY_CHARGE = new Waypoint[] {
        new Waypoint(_5, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_5_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _6H_MOBILITY_CHARGE = new Waypoint[] {
        new Waypoint(_6, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_6_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
    };
    
    _1H_A_CHARGE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_A_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _2H_A_CHARGE = new Waypoint[] {
        new Waypoint(_2, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_2_A, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_A_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _3H_A_CHARGE = new Waypoint[] {
        new Waypoint(_3, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_3_A, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_A_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _7H_D_CHARGE = new Waypoint[] {
        new Waypoint(_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_D, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_D_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _8H_D_CHARGE = new Waypoint[] {
        new Waypoint(_8, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_8_D, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_D_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _9H_D_CHARGE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D, PlacementConstants.CONE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_M, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE)
    };

    _9H_D_7H_REACH = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_REACH, PlacementConstants.LONG_ARM_REACH_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CONE),
        new Waypoint(_D_REACH_7, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_7_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _9H_D_8H_CHARGE_BLUE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_BLUE, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
    };

    _9H_D_8H_CHARGE_RED = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_RED, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED),
    };

    _9H_D_8H = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_RED, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
    };

    _9H_D_8H_C = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_RED, PlacementConstants.AUTO_CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_C, PlacementConstants.CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _9H_D_8H_C_8M_RED = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_RED, PlacementConstants.AUTO_CUBE_HIGH_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_C_8_RED, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
    };

    _9H_D_8H_C_CHARGE_RED = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_RED, PlacementConstants.AUTO_CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_C_CH_RED, PlacementConstants.AUTO_CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _9H_D_8H_C_8M_BLUE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_BLUE, PlacementConstants.AUTO_CUBE_HIGH_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_C_8_BLUE, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
    };

    _9H_D_8H_C_CHARGE_BLUE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_9_D_8_BLUE, PlacementConstants.AUTO_CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_C_CH_BLUE, PlacementConstants.AUTO_CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _1H_A_2H_B_2M_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_2_B_2_BLUE, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
    };

    _1H_A_2H_B_2M_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_RED, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_2_B_2_RED, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
    };

    _1H_A_2H_B_CHARGE_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_2_B_CH_BLUE, PlacementConstants.AUTO_CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _1H_A_2H_B_CHARGE_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_RED, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_2_B_CH_RED, PlacementConstants.AUTO_CUBE_INTAKE_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _MOBILITY_CHARGE_TEST_ONLY = new Waypoint[] {
        new Waypoint(_M_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _POOF_1H_A_2H_B_2M_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_1_A_2_RED, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_2_RED, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
    };

    _POOF_1H_A_2H_B_2M_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_2, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
    };

    _POOF_9H_D_8H_C_8M = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_9_D_8, PlacementConstants.CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_8_C_8, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE   ),
    };
    
    _SPIN = new Waypoint[] {
        new Waypoint(_1, 0, 0),
        new Waypoint(_CCWSPIN, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };


    myAutoContainer = new AutoPose[] {
      // More commonly used paths first:
      //3P 3P 3P 3P
      new AutoPose("POOF_1H_A_2H_B_2M_RED", _POOF_1H_A_2H_B_2M_RED),
      new AutoPose("POOF_1H_A_2H_B_2M_BLUE", _POOF_1H_A_2H_B_2M_BLUE),
      
      new AutoPose("POOF_9H_D_8H_C_8M_", _POOF_9H_D_8H_C_8M),
    //   new AutoPose("9H_D_8H_C_8M_RED_", _9H_D_8H_C_8M_RED),
    //   new AutoPose("9H_D_8H_C_8M_BLUE_", _9H_D_8H_C_8M_BLUE),

    new AutoPose("6H_MOBILITY_CHARGE", _6H_MOBILITY_CHARGE),
    new AutoPose("4H_ONLY", _6H_ONLY),
      
      new AutoPose("1H_A_2H_B_2M_BLUE_", _1H_A_2H_B_2M_BLUE),
      new AutoPose("1H_A_2H_B_2M_RED_", _1H_A_2H_B_2M_RED),
      
      new AutoPose("9H_D_8H_C_CHARGE_RED_", _9H_D_8H_C_CHARGE_RED),
      new AutoPose("9H_D_8H_C_CHARGE_BLUE_", _9H_D_8H_C_CHARGE_BLUE),

      new AutoPose("1H_A_2H_B_CHARGE_RED_", _1H_A_2H_B_CHARGE_RED),
      new AutoPose("1H_A_2H_B_CHARGE_BLUE_", _1H_A_2H_B_CHARGE_BLUE),
      
      // AutoPoses for the bottom of the field:
      new AutoPose("1H_A_2H_CHARGE_RED_", _1H_A_2H_CHARGE_RED),
      new AutoPose("1H_A_2H_CHARGE_BLUE_", _1H_A_2H_CHARGE_BLUE),
      new AutoPose("1H_A_2H", _1H_A_2H),
      new AutoPose("1H_A_2H_B", _1H_A_2H_B),

      new AutoPose("1H_A_3H_CHARGE", _1H_A_3H_CHARGE),
      new AutoPose("1H_A_3H", _1H_A_3H),
      new AutoPose("1H_A_3H_B", _1H_A_3H_B),

      new AutoPose("2H_A_1H_CHARGE", _2H_A_1H_CHARGE),
      new AutoPose("2H_A_1H", _2H_A_1H),
      new AutoPose("2H_A_1H_B", _2H_A_1H_B),

      new AutoPose("2H_A_3H_CHARGE", _2H_A_3H_CHARGE),
      new AutoPose("2H_A_3H", _2H_A_3H),
      new AutoPose("2H_A_3H_B", _2H_A_3H_B),

      new AutoPose("3H_A_1H_CHARGE", _3H_A_1H_CHARGE),
      new AutoPose("3H_A_1H", _3H_A_1H),
      new AutoPose("3H_A_1H_B", _3H_A_1H_B),

      new AutoPose("3H_A_2H_CHARGE", _3H_A_2H_CHARGE),
      new AutoPose("3H_A_2H", _3H_A_2H),
      new AutoPose("3H_A_2H_B", _3H_A_2H_B),

      // AutoPoses for the top of the field:
      new AutoPose("7H_D_8H_CHARGE", _7H_D_8H_CHARGE),
      new AutoPose("7H_D_8H", _7H_D_8H),
      new AutoPose("7H_D_8H_C", _7H_D_8H_C),

      new AutoPose("7H_D_9H_CHARGE", _7H_D_9H_CHARGE),
      new AutoPose("7H_D_9H", _7H_D_9H),
      new AutoPose("7H_D_9H_C", _7H_D_9H_C),

      new AutoPose("8H_D_7H_CHARGE", _8H_D_7H_CHARGE),
      new AutoPose("8H_D_7H", _8H_D_7H),
      new AutoPose("8H_D_7H_C", _8H_D_7H_C),

      new AutoPose("8H_D_9H_CHARGE", _8H_D_9H_CHARGE),
      new AutoPose("8H_D_9H", _8H_D_9H),
      new AutoPose("8H_D_9H_C", _8H_D_9H_C),

      new AutoPose("9H_D_7H_CHARGE", _9H_D_7H_CHARGE),
      new AutoPose("9H_D_7H", _9H_D_7H),
      new AutoPose("9H_D_7H_C", _9H_D_7H_C),
      
      new AutoPose("1H_A_CHARGE", _1H_A_CHARGE),
      new AutoPose("2H_A_CHARGE", _2H_A_CHARGE),
      new AutoPose("3H_A_CHARGE", _3H_A_CHARGE),

      new AutoPose("1H_A", _1H_A),
      new AutoPose("2H_A", _2H_A),
      new AutoPose("3H_A", _3H_A),

      new AutoPose("7H_D", _7H_D),
      new AutoPose("8H_D", _8H_D),
      new AutoPose("9H_D", _9H_D),

      new AutoPose("4H_MOBILITY", _4H_MOBILITY),
      new AutoPose("5H_MOBILITY", _5H_MOBILITY),
      new AutoPose("6H_MOBILITY", _6H_MOBILITY),

      new AutoPose("4H_MOBILITY_CHARGE", _4H_MOBILITY_CHARGE),
      new AutoPose("5H_MOBILITY_CHARGE", _5H_MOBILITY_CHARGE),
      // Me when the 6H mobility charge auto is defined at the top
      // and not with its family...
      new AutoPose("7H_D_CHARGE", _7H_D_CHARGE),
      new AutoPose("8H_D_CHARGE", _8H_D_CHARGE),
      new AutoPose("9H_D_CHARGE", _9H_D_CHARGE),

      new AutoPose("9H_D_8H_CHARGE_BLUE_", _9H_D_8H_CHARGE_BLUE),
      new AutoPose("9H_D_8H_CHARGE_RED_", _9H_D_8H_CHARGE_RED),
      new AutoPose("9H_D_8H_", _9H_D_8H),
      new AutoPose("9H_D_8H_C_", _9H_D_8H_C),

      new AutoPose("SPINCCW", _SPIN),

    };

    for (int i = 0; i < myAutoContainer.length; i++) {
      
      if (i == 0) {
        AutoPose MobilityAutoPose = new AutoPose("MOBILITY ONLY!!!!!!", _MOBILITY_ONLY);
        DriverUI.autoChooser.addOption(MobilityAutoPose.getName(), MobilityAutoPose);
        DriverUI.autoChooser.addOption(" ".repeat(i), MobilityAutoPose);
      }
      
      AutoPose AutoPose = myAutoContainer[i];
      
      DriverUI.autoChooser.addOption(AutoPose.getName(), AutoPose);

      // Every third index, add a spacer option for simplicity
      if ((i == 1 || i == 2 || i == 4 || i == 6 || i == 8 || i == 10) || ((i + 1) % 3 == 0 && i > 10) && (i != myAutoContainer.length - 1)) {
        DriverUI.autoChooser.addOption(" " + " ".repeat(i), AutoPose);
      }
    }
  }

  // The waypoint class is used to create a waypoint set
  // This is so we can create a larger path using multiple waypoints
  public static class Waypoint {

    private final PathPlannerTrajectory pathPlannerSegment;
    private final int armPosIndex;
    private final double clawDirection;

    public Waypoint(PathPlannerTrajectory _PPS, int _index, double _clawDirection) {
      armPosIndex = _index;
      clawDirection = _clawDirection;
      pathPlannerSegment = _PPS;
    }

    public PathPlannerTrajectory getPathPlannerSegment() {
      return pathPlannerSegment;
    }

    public int getArmPosIndex() {
      return armPosIndex;
    }

    public double getClawDirection() {
      return clawDirection;
    }
  }

  // The auto pose class is used to create a waypoint set with a string
  // This is so we can have it in a drop-down menu in shuffleboard
  @SuppressWarnings({"CanBeFinal", "SameParameterValue"})
  public static class AutoPose {
    
    private final String name;
    private final Waypoint[] thisWPset;

    AutoPose(String _S, Waypoint[] _WP) {
      thisWPset = _WP;
      name = _S;
    }

    public String getName() {
      return name;
    }

    public Waypoint[] getWaypointSet() {
      return thisWPset;
    }
  }
}