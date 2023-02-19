// Kudos to https://youtu.be/IKOGwoJ2HLk for the theory!
package math;

import math.Constants.*;

public class ArmCalculations {

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
        double upperAngle =
                (Math.acos(
                        (((Math.pow(x, 2)) + (Math.pow(y, 2))) - 
                            ((Math.pow(ArmConstants.LOWER_ARM_LENGTH, 2)) +
                                (Math.pow(ArmConstants.UPPER_ARM_LENGTH, 2)))) / 
                        (2 * (ArmConstants.LOWER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH))));

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
        double alpha = Math.atan2(
                (ArmConstants.UPPER_ARM_LENGTH * Math.sin(q2)) ,
                (ArmConstants.LOWER_ARM_LENGTH + (ArmConstants.UPPER_ARM_LENGTH * Math.cos(q2))));

        double q1 = ((Math.atan2(y , x) + alpha) - (Math.PI / 2));
        return q1;
    }
}