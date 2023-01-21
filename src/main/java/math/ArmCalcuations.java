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
     * Please keep in mind the x and y value must be under Constants.maxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getQ2 (double x, double y)
    {

        // Due to x and y being inputed as xbox contoller axis values
        // They will be only from -1 to 1
        // Thus, we multiply them by the max reach to scale them for the arm
        x *= Constants.ArmConstants.maxReachX;
        y *= Constants.ArmConstants.maxReachY;
        
        // Fail safe in case the X goes over max reach
        if (x > Constants.ArmConstants.maxReachX)
            x = Constants.ArmConstants.maxReachX;

        // Fail safe in case the Y goes over max reach
        if (y > Constants.ArmConstants.maxReachY)
            y = Constants.ArmConstants.maxReachY;

        
        double q2 = Math.acos (
            (x * x + y * y) - (Constants.ArmConstants.a1 * Constants.ArmConstants.a1 + Constants.ArmConstants.a2 * Constants.ArmConstants.a2)
            / (2 * Constants.ArmConstants.a1 * Constants.ArmConstants.a2));
        
        // Need to code

        /**
         * Check if the second segment of the arm is left or right of the first arm
         * if true, multiply q2 by -1
         */

        return q2;
    }


    /**
     * Get the offset of the first arm relative to the robot
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.maxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getQ1 (double x, double y, double q2)
    {

        // Due to x and y being inputed as xbox contoller axis values
        // They will be only from -1 to 1
        // Thus, we multiply them by the max reach to scale them for the arm
        x *= Constants.ArmConstants.maxReachX;
        y *= Constants.ArmConstants.maxReachY;
        
        // Fail safe in case the X goes over max reach
        if (x > Constants.ArmConstants.maxReachX)
            x = Constants.ArmConstants.maxReachX;

        // Fail safe in case the Y goes over max reach
        if (y > Constants.ArmConstants.maxReachY)
            y = Constants.ArmConstants.maxReachY;


        double q1Left = Math.atan(y / x) + 
                        Math.atan((Constants.ArmConstants.a2 * Math.sin(q2))
                                / (Constants.ArmConstants.a1 + Constants.ArmConstants.a2*Math.cos(q2)));
        

        
        double q1Right = Math.atan(y / x) - 
                        Math.atan((Constants.ArmConstants.a2 * Math.sin(q2))
                                /(Constants.ArmConstants.a1 + Constants.ArmConstants.a2*Math.cos(q2)));
        
        return q1Left;

    }

    /**
     * Get the distance from a point using pythag theorem
     * @param x absolute x position
     * @param y absolute y position
     * @return sqrt of the two added together
     */
    public double getDistance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    // /**
    //  * Use the law of cos to find length c
    //  * @param a the length of the first segment of the arm
    //  * @param b the length of the second segment of the arm
    //  * @param C The angle between a and b
    //  * @return the distance from the two segments of the arm
    //  */
    // public double lawOfCosines(double a, double b, double C) {
    //     return Math.acos(
    //         (a * a + b * b - C * C) / 
    //         (2 * a * b));
    // }
    //
    //
    // /**
    //  * Get the angles needed to get to a location
    //  * @param x absolute x position
    //  * @param y absolute y position
    //  * @return angles needed for joints
    //  */
    // public double[] getAngles(double x, double y)
    // {
    //     double[] angles = new double[2];

    //     double dist = getDistance(x, y);

    //     double D1 = Math.atan2(y, x);
    //     double D2 = lawOfCosines(dist, a, b);

    //     angles[0] = D1 + D2;
    //     angles[1] = lawOfCosines(a, b, dist);

    //     return angles;
    // }
    
}