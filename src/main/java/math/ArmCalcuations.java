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
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     * @return the angle to set the motor to, in radians
     */
    public double getLowerAngle(double y, double x)
    {
      x *= ArmConstants.kMaxReachX;
      y *= ArmConstants.kMaxReachY;
      
      if (x > ArmConstants.kMaxReachX) {
        x = ArmConstants.kMaxReachX;
      }
      if (y > ArmConstants.kMaxReachY) {
        y = ArmConstants.kMaxReachY;
      }

      double lowerAngle = 
        Math.acos(
          (((Math.pow(x, 2)) + (Math.pow(y, 2))) - ((Math.pow(ArmConstants.kLowerArmLength, 2)) + 
              (Math.pow(ArmConstants.kUpperArmLength, 2))))
          / (2 * (ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength)));

      return lowerAngle * ((x < 0) ? 1 : -1);
    }


    /**
     * Get the offset of the first arm relative to the robot
     * @param x the absolute x position from the first pivot point to position the claw
     * @param y the absolute y position from the first pivot point to position the claw
     * Please keep in mind the x and y value must be under Constants.kMaxReachX,Y respectivly
     * Due to an axis controlling the range, they will not go over
     */
    public double getUpperAngle(double y, double x, double q2)
    {
      x *= ArmConstants.kMaxReachX;
      y *= ArmConstants.kMaxReachY;

      if (x > ArmConstants.kMaxReachX) {
        x = ArmConstants.kMaxReachX;
      }
      if (y > ArmConstants.kMaxReachY) {
        y = ArmConstants.kMaxReachY;
      }

      double leftAngle = Math.atan(y / x) + 
              Math.atan((ArmConstants.kUpperArmLength * Math.sin(q2))
                      /(ArmConstants.kLowerArmLength + (ArmConstants.kUpperArmLength * Math.cos(q2))));

      double rightAngle = Math.atan(y / x) -
              Math.atan((ArmConstants.kUpperArmLength * Math.sin(q2))
                      / (ArmConstants.kLowerArmLength + (ArmConstants.kUpperArmLength * Math.cos(q2))));

      // leftAngle must correspond with a positive getLowerAngle() output.
      return ((x < 0) ? leftAngle : rightAngle);
    }
}