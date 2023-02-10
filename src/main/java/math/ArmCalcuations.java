package math;

import math.Constants.*;

public class ArmCalcuations {

    /*
     *   |
     *   |         C
     *   |         *
     *   |     a /   \ b
     * x |      /     \
     *   |    A/_______\B
     *   |         c
     *   |_____________________
     *             y
     */

    /**
     * Get the offset of the second arm relative to arm 1
     *
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     *          Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     *          Due to an axis controlling the range, they will not go over
     * @return the angle to set the motor to, in radians
     */
    public double getUpperAngle(double y, double x)
    {
      
        // Make sure the values are not over the max reach (-1)
        // The -1 is applied as a bufffer to prevent the math from going over
        // (just in case)
        if (x >= ArmConstants.kMaxReach - 1) {
          x = ArmConstants.kMaxReach - 1;
        }
        if (y >= ArmConstants.kMaxReach - 1) {
          y = ArmConstants.kMaxReach - 1;
        }

        if (x >= ArmConstants.kMaxReach) {
            x = ArmConstants.kMaxReach - 1;
        }
        if (y >= ArmConstants.kMaxReach) {
            y = ArmConstants.kMaxReach - 1;
        }

        double upperAngle =
                (Math.acos(
                        (((Math.pow(x, 2)) + (Math.pow(y, 2))) - ((Math.pow(ArmConstants.kLowerArmLength, 2)) +
                                (Math.pow(ArmConstants.kUpperArmLength, 2))))
                                / (2 * (ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength))));

        return upperAngle * ((x > 0) ? 1 : -1);
    }


    /**
     * Get the offset of the first arm relative to the robot
     *
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     *          Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     *          Due to an axis controlling the range, they will not go over
     */
    public double getLowerAngle(double y, double x, double q2)
    {

        // Make sure the values are not over the max reach (-1)
        // The -1 is applied as a bufffer to prevent the math from going over
        // (just in case)
        if (x >= ArmConstants.kMaxReach - 1) {
          x = ArmConstants.kMaxReach - 1;
        }
        if (y >= ArmConstants.kMaxReach - 1) {
          y = ArmConstants.kMaxReach - 1;
        }

        double leftAngle = Math.atan(y / x) +
                Math.atan((ArmConstants.kUpperArmLength * Math.sin(q2))
                        / (ArmConstants.kLowerArmLength + (ArmConstants.kUpperArmLength * Math.cos(q2))));

        double rightAngle = Math.atan(y / x) -
                Math.atan((ArmConstants.kUpperArmLength * Math.sin(q2))
                        / (ArmConstants.kLowerArmLength + (ArmConstants.kUpperArmLength * Math.cos(q2))));

        return rightAngle; // ((x < 0) ? leftAngle : rightAngle);
    }
}