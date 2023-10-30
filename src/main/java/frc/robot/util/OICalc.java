package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Constants.OIConstants;

public class OICalc {

    private static boolean driverUp = false;
    private static boolean driverDown = false;
    private static boolean driverLeft = false;
    private static boolean driverRight = false;

    private static boolean operatorUp = false;
    private static boolean operatorDown = false;
    private static boolean operatorLeft = false;
    private static boolean operatorRight = false;

    // All calculations can be referenced here https://www.desmos.com/calculator/e07raajzh5
    public static Translation2d toCircle(double x, double y) {

        Translation2d intercept;
        Translation2d output;

        double slope = y / x;

        if (slope == 0 || Double.isNaN(slope) || Double.isInfinite(slope)) {

            output = new Translation2d(x, y);
            return output;

        } else if (0 > slope && slope >= -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(-1, -slope);

        } else if (-OIConstants.CONTROLLER_CORNER_SLOPE_1 < slope && slope < -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope - 1);
            double intersectionY = (intersectionX + 1.7);

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (slope < -OIConstants.CONTROLLER_CORNER_SLOPE_1 || slope > OIConstants.CONTROLLER_CORNER_SLOPE_1) {

            intercept = new Translation2d(1 / slope, 1);

        } else if (OIConstants.CONTROLLER_CORNER_SLOPE_1 > slope && slope > OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope + 1);
            double intersectionY = -intersectionX + 1.7;

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (0 < slope && slope <= OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(1, slope);

        } else {

            intercept = new Translation2d(0, 0);

            System.out.println("Error... Slope = " + slope);

        }

        double distance = getDistance(x, y, intercept.getX(), intercept.getY());

        distance = (distance > 1) ? 1 : distance;

        output = new Translation2d(Math.signum(x) * distance, new Rotation2d(Math.atan(y / x)));

        return output;
    }

    public static double getDistance(double x1, double y1, double x2, double y2) {
        return
                Math.sqrt(
                        (Math.pow(x1, 2) + Math.pow(y1, 2)) /
                        (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

    public static int getDriverPOVPressed(int POV) {
      
      int pressedPOV = -1;
      
      if (POV == 0) { 
        if (!driverUp) {
          driverUp = true;
          pressedPOV = 0;
        }
      }
      else {
        driverUp = false;
      }

      if (POV == 180) { 
        if (!driverDown) {
          driverDown = true;
          pressedPOV = 180;
        }
      }
      else {
        driverDown = false;
      }

      if (POV == 270) { 
        if (!driverLeft) {
          driverLeft = true;
          pressedPOV = 270;
        }
      }
      else {
        driverLeft = false;
      }

      if (POV == 90) { 
        if (!driverRight) {
          driverRight = true;
          pressedPOV = 90;
        }
      }
      else {
        driverRight = false;
      }

      return pressedPOV;
    }

    public static int getOperatorPOVPressed(int POV) {

      int pressedPOV = -1;
      
      if (POV == 0) { 
        if (!operatorUp) {
          operatorUp = true;
          pressedPOV = 0;
        }
      }
      else {
        operatorUp = false;
      }

      if (POV == 180) { 
        if (!operatorDown) {
          operatorDown = true;
          pressedPOV = 180;
        }
      }
      else {
        operatorDown = false;
      }

      if (POV == 270) { 
        if (!operatorLeft) {
          operatorLeft = true;
          pressedPOV = 270;
        }
      }
      else {
        operatorLeft = false;
      }

      if (POV == 90) { 
        if (!operatorRight) {
          operatorRight = true;
          pressedPOV = 90;
        }
      }
      else {
        operatorRight = false;
      }

      return pressedPOV;
    }
  }
