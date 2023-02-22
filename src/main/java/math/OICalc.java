package math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import math.Constants.OIConstants;

public class OICalc {

    private static boolean isClickingUp = false;
    private static boolean isHoldingUp = false;

    private static boolean isClickingDown = false;
    private static boolean isHoldingDown = false;

    private static boolean isClickingLeft = false;
    private static boolean isHoldingLeft = false;

    private static boolean isClickingRight = false;
    private static boolean isHoldingRight = false;

    // All calculations can be refrenced here https://www.desmos.com/calculator/e07raajzh5
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

    public static int getPOVPressed(int POV) {
      if (POV == 0) { 
        if (!isHoldingUp) {
          isClickingUp = true;
        }
        else {
          isClickingUp = false;
        }
        isHoldingUp = true;
      }
      else {
        isClickingUp = false;
        isHoldingUp = false;
      }

      if (POV == 180) { 
        if (!isHoldingDown) {
          isClickingDown = true;
        }
        else {
          isClickingDown = false;
        }
        isHoldingDown = true;
      }
      else {
        isClickingDown = false;
        isHoldingDown = false;
      }

      if (POV == 270) { 
        if (!isHoldingLeft) {
          isClickingLeft = true;
        }
        else {
          isClickingLeft = false;
        }
        isHoldingLeft = true;
      }
      else {
        isClickingLeft = false;
        isHoldingLeft = false;
      }

      if (POV == 90) { 
        if (!isHoldingRight) {
          isClickingRight = true;
        }
        else {
          isClickingRight = false;
        }
        isHoldingRight = true;
      }
      else {
        isClickingRight = false;
        isHoldingRight = false;
      }
      
      if (isClickingUp) {
        return 0;
      }
      else if (isClickingDown) {
        return 180;
      }
      else if (isClickingLeft) {
        return 270;
      }
      else if (isClickingRight) {
        return 90;
      }
      else {
        return -1;
      }
    }

  }
