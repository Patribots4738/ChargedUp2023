package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import frc.robot.DriverUI;
import frc.robot.util.Constants.AutoConstants;

public class AutoPathStorage {
    public static PathPlannerTrajectory[] chosenWaypoints;

    public static AutoMap[] myAutoContainer;


    /*
     * Naming convention:
     * 1-9 represents the grid index starting from the bottom of the field in
     * pathplanner
     * A-D represents the game piece index starting from the bottom of the field in
     * pathplanner
     * CHARGE represents if the robot is going onto the charge pad
     * if CHARGE is excluded, the robot will not go onto the charge pad
     * H after 1-9 represents the high placement index for the arm
     * M after 1-9 represents the mid placement index for the arm
     * L after 1-9 represents the hybrid placement index for the arm
     */

    public static PathPlannerTrajectory[] _MOBILITY_ONLY;

    public static PathPlannerTrajectory[] _SQUARE_HALF;

    // Paths for the bottom of the field:
    // Start at grid index 1, place high, then go to field element A, intake, then
    // go to grid index 2, and place high
    public static PathPlannerTrajectory[] _1H_A_2H_CHARGE_BLUE;
    public static PathPlannerTrajectory[] _1H_A_2H_CHARGE_RED;
    public static PathPlannerTrajectory[] _1H_A_2H;
    public static PathPlannerTrajectory[] _1H_A_2H_B;

    // Start at grid index 1, place high, then go to field element A, intake, then
    // go to grid index 3, and place high
    public static PathPlannerTrajectory[] _1H_A;
    public static PathPlannerTrajectory[] _1H_A_3H_CHARGE;
    public static PathPlannerTrajectory[] _1H_A_3H;
    public static PathPlannerTrajectory[] _1H_A_3H_B;

    // Start at grid index 2, place high, then go to field element A, intake, then
    // go to grid index 1, and place high
    public static PathPlannerTrajectory[] _2H_A_1H_CHARGE;
    public static PathPlannerTrajectory[] _2H_A_1H;
    public static PathPlannerTrajectory[] _2H_A;
    public static PathPlannerTrajectory[] _2H_A_1H_B;

    // Start at grid index 2, place high, then go to field element A, intake, then
    // go to grid index 3, and place high
    public static PathPlannerTrajectory[] _2H_A_3H_CHARGE;
    public static PathPlannerTrajectory[] _2H_A_3H;
    public static PathPlannerTrajectory[] _2H_A_3H_B;

    // Start at grid index 3, place high, then go to field element A, intake, then
    // go to grid index 1, and place high
    public static PathPlannerTrajectory[] _3H_A_1H_CHARGE;
    public static PathPlannerTrajectory[] _3H_A_1H;
    public static PathPlannerTrajectory[] _3H_A;
    public static PathPlannerTrajectory[] _3H_A_1H_B;

    // Start at grid index 3, place high, then go to field element A, intake, then
    // go to grid index 2, and place high
    public static PathPlannerTrajectory[] _3H_A_2H_CHARGE;
    public static PathPlannerTrajectory[] _3H_A_2H;
    public static PathPlannerTrajectory[] _3H_A_2H_B;

    // Paths for the top of the field:
    // Start at grid index 7, place high, then go to field element D, intake, then
    // go to grid index 8, and place high
    public static PathPlannerTrajectory[] _7H_D_8H_CHARGE;
    public static PathPlannerTrajectory[] _7H_D_8H;
    public static PathPlannerTrajectory[] _7H_D;
    public static PathPlannerTrajectory[] _7H_D_8H_C;

    // Start at grid index 7, place high, then go to field element D, intake, then
    // go to grid index 9, and place high
    public static PathPlannerTrajectory[] _7H_D_9H_CHARGE;
    public static PathPlannerTrajectory[] _7H_D_9H;
    public static PathPlannerTrajectory[] _7H_D_9H_C;

    // Start at grid index 8, place high, then go to field element D, intake, then
    // go to grid index 7, and place high
    public static PathPlannerTrajectory[] _8H_D_7H_CHARGE;
    public static PathPlannerTrajectory[] _8H_D_7H;
    public static PathPlannerTrajectory[] _8H_D;
    public static PathPlannerTrajectory[] _8H_D_7H_C;

    // Start at grid index 8, place high, then go to field element D, intake, then
    // go to grid index 9, and place high
    public static PathPlannerTrajectory[] _8H_D_9H_CHARGE;
    public static PathPlannerTrajectory[] _8H_D_9H;
    public static PathPlannerTrajectory[] _8H_D_9H_C;

    // Start at grid index 9, place high, then go to field element D, intake, then
    // go to grid index 7, and place high
    public static PathPlannerTrajectory[] _9H_D_7H_CHARGE;
    public static PathPlannerTrajectory[] _9H_D_7H;
    public static PathPlannerTrajectory[] _9H_D;
    public static PathPlannerTrajectory[] _9H_D_7H_C;

    // Start at grid index 9, place high, then go to field element D, intake, then
    // go to grid index 8, and place high
    public static PathPlannerTrajectory[] _9H_D_8H_CHARGE_BLUE;
    public static PathPlannerTrajectory[] _9H_D_8H_CHARGE_RED;
    public static PathPlannerTrajectory[] _9H_D_8H;
    public static PathPlannerTrajectory[] _9H_D_8H_C;

    public static PathPlannerTrajectory[] _9H_D_8H_C_8M_RED;
    public static PathPlannerTrajectory[] _9H_D_8H_C_CHARGE_RED;

    public static PathPlannerTrajectory[] _9H_D_8H_C_8M_BLUE;
    public static PathPlannerTrajectory[] _POOF_9H_D_8H_C_CHARGE;

    public static PathPlannerTrajectory[] _1H_A_2H_B_2M_RED;
    public static PathPlannerTrajectory[] _POOF_1H_A_2H_B_CHARGE_RED;

    public static PathPlannerTrajectory[] _1H_A_2H_B_2M_BLUE;
    public static PathPlannerTrajectory[] _POOF_1H_A_2H_B_CHARGE_BLUE;

    public static PathPlannerTrajectory[] _4H_MOBILITY;
    public static PathPlannerTrajectory[] _5H_MOBILITY;
    public static PathPlannerTrajectory[] _6H_MOBILITY;
    public static PathPlannerTrajectory[] _6H_ONLY;
    public static PathPlannerTrajectory[] _4H_MOBILITY_CHARGE;
    public static PathPlannerTrajectory[] _5H_MOBILITY_CHARGE;
    public static PathPlannerTrajectory[] _6H_MOBILITY_CHARGE;

    public static PathPlannerTrajectory[] _1H_A_CHARGE;
    public static PathPlannerTrajectory[] _2H_A_CHARGE;
    public static PathPlannerTrajectory[] _3H_A_CHARGE;
    public static PathPlannerTrajectory[] _7H_D_CHARGE;
    public static PathPlannerTrajectory[] _8H_D_CHARGE;
    public static PathPlannerTrajectory[] _9H_D_CHARGE;

    public static PathPlannerTrajectory[] _9H_D_7H_REACH;
    public static PathPlannerTrajectory[] _MOBILITY_CHARGE_TEST_ONLY;

    public static PathPlannerTrajectory[] _POOF_1H_A_2H_B_2MID_BLUE;
    public static PathPlannerTrajectory[] _POOF_1H_A_2H_B_2LOW_BLUE;
    public static PathPlannerTrajectory[] _POOF_1H_A_2H_B_2MID_RED;
    public static PathPlannerTrajectory[] _POOF_1H_A_2H_B_2LOW_RED;
    public static PathPlannerTrajectory[] _POOF_9H_D_8H_C_8MID;
    public static PathPlannerTrajectory[] _POOF_9H_D_8H_C_8LOW;

    public static PathPlannerTrajectory[] _SPIN;

    public static PathPlannerTrajectory square1;
    public static PathPlannerTrajectory square2;
    public static PathPlannerTrajectory square3;
    public static PathPlannerTrajectory square4;

    public static PathPlannerTrajectory _MOBILITY;

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

    public static PathPlannerTrajectory _POOF_2_B_CH_BLUE;
    public static PathPlannerTrajectory _POOF_2_B_CH_RED;
    public static PathPlannerTrajectory _POOF_8_C_CH;

    public static PathPlannerTrajectory _CCWSPIN;

    /**
     * Create all the waypoints needed for each path
     * then, after all has been completed, add them to the myAutoContainer array
     */
    public AutoPathStorage() {

        loadAutoPaths();

        generateWaypointArrays();

        myAutoContainer = new AutoMap[] {
                // More commonly used paths first:
                // 3P 3P 3P 3P
                new AutoMap("POOF 1A2B2 MID RED", _POOF_1H_A_2H_B_2MID_RED),
                new AutoMap("POOF 1A2B2 LOW RED", _POOF_1H_A_2H_B_2LOW_RED),
                new AutoMap("POOF 1A2B2 MID BLUE", _POOF_1H_A_2H_B_2MID_BLUE),
                new AutoMap("POOF 1A2B2 LOW BLUE", _POOF_1H_A_2H_B_2LOW_BLUE),

                new AutoMap("POOF 9D8C8 MID", _POOF_9H_D_8H_C_8MID),
                new AutoMap("POOF 9D8C8 LOW", _POOF_9H_D_8H_C_8LOW),

                new AutoMap("6H_MOBILITY_CHARGE", _6H_MOBILITY_CHARGE),
                new AutoMap("4H_ONLY", _6H_ONLY),

                new AutoMap("POOF 1A2 B CHARGE RED", _POOF_1H_A_2H_B_CHARGE_RED),
                new AutoMap("POOF 1A2 B CHARGE BLUE", _POOF_1H_A_2H_B_CHARGE_BLUE),

                new AutoMap("POOF 9D8 C CHARGE", _POOF_9H_D_8H_C_CHARGE),

        };

        for (int i = 0; i < myAutoContainer.length; i++) {

            if (i == 8) {
                AutoMap MobilityAutoPose = new AutoMap("MOBILITY ONLY!!!!!!", _MOBILITY_ONLY);
                DriverUI.autoChooser.addOption(MobilityAutoPose.getName(), MobilityAutoPose);
                DriverUI.autoChooser.addOption(" ".repeat(i), MobilityAutoPose);
            }

            AutoMap autoMap = myAutoContainer[i];

            DriverUI.autoChooser.addOption(autoMap.getName(), autoMap);

            // Every third index, add a spacer option for simplicity
            if ((i == 1 || i == 3 || i == 5 || i == 9 || i == 10)
                    || ((i + 2) % 3 == 0 && i > 10) && (i != myAutoContainer.length - 1)) {
                DriverUI.autoChooser.addOption(" " + " ".repeat(i), autoMap);
            }
        }
    }

    private void generateWaypointArrays() {
        /*
         * The syntax here is as follows:
         * PATH NAME = new PathPlannerTrajectory[] {
         * new Waypoint(
         * PATH,
         * ARM_PLACEMENT INDEX,
         * CLAW_SPEED
         * ),
         * new Waypoint(
         * PATH_2,
         * ARM_PLACEMENT INDEX,
         * CLAW_SPEED
         * )
         * ...
         * }
         * If the path is 1/3/7/9, then the arm will go to a cone index
         * If the path is 2/4/5/8, then the arm will go to a cube index
         * When going to a number, the claw will outtake, and the arm will go to a
         * placement position
         * H = high, M = mid, L = hybrid
         * When going to a letter, the claw will intake, and the arm will go to the
         * floor intake
         * When going to a charge, the claw will stop, and arm stowed
         * Create waypoint arrays in order as defined from the top of the file
         */

        _SQUARE_HALF = new PathPlannerTrajectory[] {
                square1,
                square2
        };

        _MOBILITY_ONLY = new PathPlannerTrajectory[] {
                _MOBILITY
        };

        _4H_MOBILITY = new PathPlannerTrajectory[] {
                _4_M_CH
        };

        _6H_MOBILITY = new PathPlannerTrajectory[] {
                _6_M_CH
        };

        _4H_MOBILITY_CHARGE = new PathPlannerTrajectory[] {
                _4_M_CH
        };

        _5H_MOBILITY_CHARGE = new PathPlannerTrajectory[] {
                _5_M,
                _M_CH
        };

        _6H_MOBILITY_CHARGE = new PathPlannerTrajectory[] {
                _6_M_CH,
        };

        _9H_D_8H_C = new PathPlannerTrajectory[] {
                _9_D_8_RED,
                _8_C,
        };

        _9H_D_8H_C_CHARGE_RED = new PathPlannerTrajectory[] {
                _9_D_8_RED,
                _8_C_CH_RED,
        };

        _POOF_9H_D_8H_C_CHARGE = new PathPlannerTrajectory[] {
                _POOF_9_D_8,
                _POOF_8_C_CH,
        };

        _POOF_1H_A_2H_B_CHARGE_BLUE = new PathPlannerTrajectory[] {
                _1_A_2_BLUE,
                _POOF_2_B_CH_BLUE,
        };

        _POOF_1H_A_2H_B_CHARGE_RED = new PathPlannerTrajectory[] {
                _POOF_1_A_2_RED,
                _POOF_2_B_CH_RED,
        };

        _POOF_1H_A_2H_B_2MID_RED = new PathPlannerTrajectory[] {
                _POOF_1_A_2_RED,
                _POOF_2_B_2_RED,
                _2_B
        };

        _POOF_1H_A_2H_B_2MID_BLUE = new PathPlannerTrajectory[] {
                _1_A_2_BLUE,
                _POOF_2_B_2,
                _2_B
        };

        _POOF_9H_D_8H_C_8MID = new PathPlannerTrajectory[] {
                _POOF_9_D_8,
                _POOF_8_C_8,
                _8_D
        };

        _POOF_1H_A_2H_B_2LOW_RED = new PathPlannerTrajectory[] {
                _POOF_1_A_2_RED,
                _POOF_2_B_2_RED,
                _2_B
        };

        _POOF_1H_A_2H_B_2LOW_BLUE = new PathPlannerTrajectory[] {
                _1_A_2_BLUE,
                _POOF_2_B_2,
                _2_B
        };

        _POOF_9H_D_8H_C_8LOW = new PathPlannerTrajectory[] {
                _POOF_9_D_8,
                _POOF_8_C_8,
                _8_D
        };

        _SPIN = new PathPlannerTrajectory[] {
                _CCWSPIN
        };
    }

    private void loadAutoPaths() {
        square1 = PathPlanner.loadPath("Square1", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        square2 = PathPlanner.loadPath("Square2", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        square3 = PathPlanner.loadPath("Square3", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        square4 = PathPlanner.loadPath("Square4", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _MOBILITY = PathPlanner.loadPath("_MOBILITY", AutoConstants.MAX_SPEED_METERS_PER_SECOND / 2.0,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED / 2.0);

        _1_A = PathPlanner.loadPath("_1_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _1_B = PathPlanner.loadPath("_1_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _2_B = PathPlanner.loadPath("_2_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _2_A = PathPlanner.loadPath("_2_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _3_A = PathPlanner.loadPath("_3_A", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _3_B = PathPlanner.loadPath("_3_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _4_B = PathPlanner.loadPath("_4_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _4_M_CH = PathPlanner.loadPath("_4_M", 2, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _5_B = PathPlanner.loadPath("_5_B", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _5_C = PathPlanner.loadPath("_5_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _5_M = PathPlanner.loadPath("_5_M", 2, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _6_C = PathPlanner.loadPath("_6_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _6_M_CH = PathPlanner.loadPath("_6_M", 2, 1);
        _M_CH = PathPlanner.loadPath("_M_CH", 3, 1);
        _7_C = PathPlanner.loadPath("_7_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _7_D = PathPlanner.loadPath("_7_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _8_C = PathPlanner.loadPath("_8_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _8_D = PathPlanner.loadPath("_8_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _9_C = PathPlanner.loadPath("_9_C", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _9_D = PathPlanner.loadPath("_9_D", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _9_D_8_RED = PathPlanner.loadPath("9_D_8_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _9_D_8_BLUE = PathPlanner.loadPath("9_D_8_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _8_C_8_RED = PathPlanner.loadPath("8_C_8_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _8_C_CH_RED = PathPlanner.loadPath("8_C_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _1_A_2_BLUE = PathPlanner.loadPath("1_A_2_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _1_A_2_RED = PathPlanner.loadPath("1_A_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _2_B_2_BLUE = PathPlanner.loadPath("2_B_2_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _2_B_2_RED = PathPlanner.loadPath("2_B_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _8_C_8_BLUE = PathPlanner.loadPath("8_C_8_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _8_C_CH_RED = PathPlanner.loadPath("8_C_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _POOF_2_B_CH_RED = PathPlanner.loadPath("_POOF_2_B_CH_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _POOF_2_B_CH_BLUE = PathPlanner.loadPath("_POOF_2_B_CH_BLUE", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _POOF_8_C_CH = PathPlanner.loadPath("_POOF_8_C_CH", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _POOF_1_A_2 = PathPlanner.loadPath("_POOF_1_A_2", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _POOF_2_B_2 = PathPlanner.loadPath("_POOF_2_B_2", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _POOF_1_A_2_RED = PathPlanner.loadPath("_POOF_1_A_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        _POOF_2_B_2_RED = PathPlanner.loadPath("_POOF_2_B_2_RED", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _POOF_9_D_8 = PathPlanner.loadPath("_POOF_9_D_8", 2.75, 3.3);
        _POOF_8_C_8 = PathPlanner.loadPath("_POOF_8_C_8", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        _CCWSPIN = PathPlanner.loadPath("SPIN", AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    }
    // The auto pose class is used to create a waypoint set with a string
    // This is so we can have it in a drop-down menu in shuffleboard
    @SuppressWarnings({ "CanBeFinal", "SameParameterValue" })
    public static class AutoMap {

        private final String name;
        private final PathPlannerTrajectory[] trajectories;

        AutoMap(String _S, PathPlannerTrajectory[] trajectories) {
            this.trajectories = trajectories;
            name = _S;
        }

        public String getName() {
            return name;
        }

        public PathPlannerTrajectory[] getTrajectories() {
            return trajectories;
        }
    }
}