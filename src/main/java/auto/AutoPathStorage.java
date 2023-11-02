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

public class AutoPathStorage {

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

  // Start at grid index 9, place high, then go to field element D, intake, then go to grid index 8, and place high
  public static Waypoint[] _9H_D_8H_CHARGE_BLUE;
  public static Waypoint[] _9H_D_8H_CHARGE_RED;
  public static Waypoint[] _9H_D_8H;
  public static Waypoint[] _9H_D_8H_C;

  public static Waypoint[] _9H_D_8H_C_8M_RED;
  public static Waypoint[] _9H_D_8H_C_CHARGE_RED;

  public static Waypoint[] _9H_D_8H_C_8M_BLUE;
  public static Waypoint[] _POOF_9H_D_8H_C_CHARGE;
  
  public static Waypoint[] _1H_A_2H_B_2M_RED;
  public static Waypoint[] _POOF_1H_A_2H_B_CHARGE_RED;

  public static Waypoint[] _1H_A_2H_B_2M_BLUE;
  public static Waypoint[] _POOF_1H_A_2H_B_CHARGE_BLUE;

  public static Waypoint[] _4H_MOBILITY_CH;
  public static Waypoint[] _5H_MOBILITY;
  public static Waypoint[] _6H_MOBILITY_CH;
  public static Waypoint[] _6H_ONLY;
  public static Waypoint[] _4H_MOBILITY_CHARGE;
  public static Waypoint[] _5H_MOBILITY_CHARGE;
  public static Waypoint[] _6H_MOBILITY_CHARGE;

  public static Waypoint[] _9H_D_7H_REACH;
  public static Waypoint[] _MOBILITY_CHARGE_TEST_ONLY;

  public static Waypoint[] _POOF_1H_A_2H_B_2MID_BLUE;
  public static Waypoint[] _POOF_1H_A_2H_B_2LOW_BLUE;
  public static Waypoint[] _POOF_1H_A_2H_B_2MID_RED;
  public static Waypoint[] _POOF_1H_A_2H_B_2LOW_RED;
  public static Waypoint[] _POOF_9H_D_8H_C_8MID;
  public static Waypoint[] _POOF_9H_D_8H_C_8LOW;

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

  public static PathPlannerTrajectory _2_B;
  public static PathPlannerTrajectory _3_B;
  public static PathPlannerTrajectory _7_D;
  public static PathPlannerTrajectory _8_D;
  public static PathPlannerTrajectory _4_M_CH;
  public static PathPlannerTrajectory _5_M;
  public static PathPlannerTrajectory _6_M_CH;
  public static PathPlannerTrajectory _M_CH;
  
  public static PathPlannerTrajectory _POOF_1_A_2;
  public static PathPlannerTrajectory _POOF_1_A_2_RED;
  public static PathPlannerTrajectory _POOF_2_B_2;
  public static PathPlannerTrajectory _POOF_2_B_2_RED;
  public static PathPlannerTrajectory _POOF_9_D_8;
  public static PathPlannerTrajectory _POOF_8_C_8;

  public static PathPlannerTrajectory _1_A_2_BLUE;

  public static PathPlannerTrajectory _POOF_2_B_CH_BLUE;
  public static PathPlannerTrajectory _POOF_2_B_CH_RED;
  public static PathPlannerTrajectory _POOF_8_C_CH;

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

    _3_B = PathPlanner.loadPath("_3_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _4_M_CH = PathPlanner.loadPath("_4_M", 2, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _5_M = PathPlanner.loadPath("_5_M", 2, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _6_M_CH = PathPlanner.loadPath("_6_M", 2, 1);
    _M_CH = PathPlanner.loadPath("_M_CH", 3, 1);

    _1_A_2_BLUE = PathPlanner.loadPath("1_A_2_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _POOF_2_B_CH_RED = PathPlanner.loadPath("_POOF_2_B_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _POOF_2_B_CH_BLUE = PathPlanner.loadPath("_POOF_2_B_CH_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    _POOF_8_C_CH = PathPlanner.loadPath("_POOF_8_C_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

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
        new PathPoint(_POOF_1_A_2.getInitialState().poseMeters.getTranslation(), _POOF_1_A_2.getInitialState().poseMeters.getRotation(), _POOF_1_A_2.getInitialHolonomicPose().getRotation()),
        new PathPoint(_POOF_1_A_2.getInitialState().poseMeters.getTranslation(), _POOF_1_A_2.getInitialState().poseMeters.getRotation(), _POOF_1_A_2.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _2_A as the starting point for _2
    // This is so we can move our arm before we move the robot.
    _2 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_POOF_2_B_2.getInitialState().poseMeters.getTranslation(), _POOF_2_B_2.getInitialState().poseMeters.getRotation(), _POOF_2_B_2.getInitialHolonomicPose().getRotation()),
        new PathPoint(_POOF_2_B_2.getInitialState().poseMeters.getTranslation(), _POOF_2_B_2.getInitialState().poseMeters.getRotation(), _POOF_2_B_2.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _3_A as the starting point for _3
    // This is so we can move our arm before we move the robot.
    _3 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_3_B.getInitialState().poseMeters.getTranslation(), _3_B.getInitialState().poseMeters.getRotation(), _3_B.getInitialHolonomicPose().getRotation()),
        new PathPoint(_3_B.getInitialState().poseMeters.getTranslation(), _3_B.getInitialState().poseMeters.getRotation(), _3_B.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _4_B as the starting point for _4
    // This is so we can move our arm before we move the robot.
    _4 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_4_M_CH.getInitialState().poseMeters.getTranslation(), _4_M_CH.getInitialState().poseMeters.getRotation(), _4_M_CH.getInitialHolonomicPose().getRotation()),
        new PathPoint(_4_M_CH.getInitialState().poseMeters.getTranslation(), _4_M_CH.getInitialState().poseMeters.getRotation(), _4_M_CH.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _5_B as the starting point for _5
    // This is so we can move our arm before we move the robot.
    _5 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_5_M.getInitialState().poseMeters.getTranslation(), _5_M.getInitialState().poseMeters.getRotation(), _5_M.getInitialHolonomicPose().getRotation()),
        new PathPoint(_5_M.getInitialState().poseMeters.getTranslation(), _5_M.getInitialState().poseMeters.getRotation(), _5_M.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _6_C as the starting point for _6
    // This is so we can move our arm before we move the robot.
    _6 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_6_M_CH.getInitialState().poseMeters.getTranslation(), _6_M_CH.getInitialState().poseMeters.getRotation(), _6_M_CH.getInitialHolonomicPose().getRotation()),
        new PathPoint(_6_M_CH.getInitialState().poseMeters.getTranslation(), _6_M_CH.getInitialState().poseMeters.getRotation(), _6_M_CH.getInitialHolonomicPose().getRotation())
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
        new PathPoint(_POOF_8_C_8.getInitialState().poseMeters.getTranslation(), _POOF_8_C_8.getInitialState().poseMeters.getRotation(), _POOF_8_C_8.getInitialHolonomicPose().getRotation()),
        new PathPoint(_POOF_8_C_8.getInitialState().poseMeters.getTranslation(), _POOF_8_C_8.getInitialState().poseMeters.getRotation(), _POOF_8_C_8.getInitialHolonomicPose().getRotation())
    );

    // Use the initial state of _9_D as the starting point for _9
    // This is so we can move our arm before we move the robot.
    _9 = PathPlanner.generatePath
    (
        new PathConstraints(0.1, 0.1),
        new PathPoint(_POOF_9_D_8.getInitialState().poseMeters.getTranslation(), _POOF_9_D_8.getInitialState().poseMeters.getRotation(), _POOF_9_D_8.getInitialHolonomicPose().getRotation()),
        new PathPoint(_POOF_9_D_8.getInitialState().poseMeters.getTranslation(), _POOF_9_D_8.getInitialState().poseMeters.getRotation(), _POOF_9_D_8.getInitialHolonomicPose().getRotation())
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

    _6H_ONLY = new Waypoint[] {
        new Waypoint(_6, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_6, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
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

    _POOF_9H_D_8H_C_CHARGE = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_9_D_8, PlacementConstants.AUTO_CUBE_HIGH_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_8_C_CH, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _POOF_1H_A_2H_B_CHARGE_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_CH_BLUE, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _POOF_1H_A_2H_B_CHARGE_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_1_A_2_RED, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_CH_RED, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_INTAKE_SPEED_CUBE),
    };

    _POOF_1H_A_2H_B_2MID_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_1_A_2_RED, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_2_RED, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
        new Waypoint(_2_B, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _POOF_1H_A_2H_B_2MID_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_2, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
        new Waypoint(_2_B, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _POOF_9H_D_8H_C_8MID = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_9_D_8, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_8_C_8, PlacementConstants.AUTO_CUBE_MID_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_D, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _POOF_1H_A_2H_B_2LOW_RED = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_1_A_2_RED, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_2_RED, PlacementConstants.STOWED_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
        new Waypoint(_2_B, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _POOF_1H_A_2H_B_2LOW_BLUE = new Waypoint[] {
        new Waypoint(_1, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_1_A_2_BLUE, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_2_B_2, PlacementConstants.STOWED_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE_FAST),
        new Waypoint(_2_B, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    _POOF_9H_D_8H_C_8LOW = new Waypoint[] {
        new Waypoint(_9, PlacementConstants.CONE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CONE),
        new Waypoint(_POOF_9_D_8, PlacementConstants.CUBE_HIGH_PLACEMENT_INDEX, PlacementConstants.CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_POOF_8_C_8, PlacementConstants.STOWED_INDEX, PlacementConstants.AUTO_CLAW_OUTTAKE_SPEED_CUBE),
        new Waypoint(_8_D, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };
    
    _SPIN = new Waypoint[] {
        new Waypoint(_1, 0, 0),
        new Waypoint(_CCWSPIN, PlacementConstants.STOWED_INDEX, PlacementConstants.CLAW_STOPPED_SPEED)
    };

    myAutoContainer = new AutoPose[] {
      // More commonly used paths first:
      //3P 3P 3P 3P
      new AutoPose("POOF 1A2B2 MID RED", _POOF_1H_A_2H_B_2MID_RED),
      new AutoPose("POOF 1A2B2 LOW RED", _POOF_1H_A_2H_B_2LOW_RED),
      new AutoPose("POOF 1A2B2 MID BLUE", _POOF_1H_A_2H_B_2MID_BLUE),
      new AutoPose("POOF 1A2B2 LOW BLUE", _POOF_1H_A_2H_B_2LOW_BLUE),
      
      new AutoPose("POOF 9D8C8 MID", _POOF_9H_D_8H_C_8MID),
      new AutoPose("POOF 9D8C8 LOW", _POOF_9H_D_8H_C_8LOW),

      new AutoPose("6H_MOBILITY_CHARGE", _6H_MOBILITY_CHARGE),
    //   new AutoPose("6H_ONLY", _6H_ONLY),
      
      new AutoPose("POOF 1A2 B CHARGE RED", _POOF_1H_A_2H_B_CHARGE_RED),
      new AutoPose("POOF 1A2 B CHARGE BLUE", _POOF_1H_A_2H_B_CHARGE_BLUE),

      new AutoPose("POOF 9D8 C CHARGE", _POOF_9H_D_8H_C_CHARGE),

    };

    for (int i = 0; i < myAutoContainer.length; i++) {
      
      if (i == 8) {
        AutoPose MobilityAutoPose = new AutoPose("MOBILITY ONLY!!!!!!", _MOBILITY_ONLY);
        DriverUI.autoChooser.addOption(MobilityAutoPose.getName(), MobilityAutoPose);
        DriverUI.autoChooser.addOption(" ".repeat(i), MobilityAutoPose);
      }
      
      AutoPose AutoPose = myAutoContainer[i];
      
      DriverUI.autoChooser.addOption(AutoPose.getName(), AutoPose);

      // Every third index, add a spacer option for simplicity
      if ((i == 1 || i == 3 || i == 5 || i == 9 || i == 10) || ((i+2) % 3 == 0 && i > 10) && (i != myAutoContainer.length - 1)) {
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