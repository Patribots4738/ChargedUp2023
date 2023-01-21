package math;

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
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getQ2 (double x, double y)
    {

        // Due to x and y being inputed as xbox contoller axis values
        // They will be only from -1 to 1
        // Thus, we multiply them by the max reach to scale them for the arm
        x *= Constants.ArmConstants.kMaxReachX;
        y *= Constants.ArmConstants.kMaxReachY;
        
        // Fail safe in case the X goes over max reach
        if (x > Constants.ArmConstants.kMaxReachX){
          x = Constants.ArmConstants.kMaxReachX;
        }
        // Fail safe in case the Y goes over max reach
        if (y > Constants.ArmConstants.kMaxReachY) {
          y = Constants.ArmConstants.kMaxReachY;
        }
        
        double q2 = Math.acos (
            (x * x + y * y) - (Constants.ArmConstants.kLowerArmLength * Constants.ArmConstants.kLowerArmLength + Constants.ArmConstants.kUpperArmLength * Constants.ArmConstants.kUpperArmLength)
            / (2 * Constants.ArmConstants.kLowerArmLength * Constants.ArmConstants.kUpperArmLength));
        
        // Need to code

        /**
         * Check if the second segment of the arm is left or right of the first arm
         * if true, multiply q2 by -1
         */

        return q2 * ((x < 0) ? 1 : -1);
    }


    /**
     * Get the offset of the first arm relative to the robot
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getQ1 (double x, double y, double q2)
    {

        // Due to x and y being inputed as xbox contoller axis values
        // They will be only from -1 to 1
        // Thus, we multiply them by the max reach to scale them for the arm
        x *= Constants.ArmConstants.kMaxReachX;
        y *= Constants.ArmConstants.kMaxReachY;
        
        // Fail safe in case the X goes over max reach
        if (x > Constants.ArmConstants.kMaxReachX)
            x = Constants.ArmConstants.kMaxReachX;

        // Fail safe in case the Y goes over max reach
        if (y > Constants.ArmConstants.kMaxReachY)
            y = Constants.ArmConstants.kMaxReachY;


        double q1Left = Math.atan(y / x) + 
                        Math.atan((Constants.ArmConstants.kUpperArmLength * Math.sin(q2))
                                / (Constants.ArmConstants.kLowerArmLength + Constants.ArmConstants.kUpperArmLength*Math.cos(q2)));
        

        
        double q1Right = Math.atan(y / x) - 
                        Math.atan((Constants.ArmConstants.kUpperArmLength * Math.sin(q2))
                                /(Constants.ArmConstants.kLowerArmLength + Constants.ArmConstants.kUpperArmLength*Math.cos(q2)));
        
        return (x < 0) ? q1Left : q1Right;

    }
    
}