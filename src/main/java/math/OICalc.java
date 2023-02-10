package math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import math.Constants.OIConstants;

public class OICalc {


    // All calculations can be refrenced here https://www.desmos.com/calculator/e07raajzh5
    public static Translation2d toCircle(double x, double y) {

        Translation2d intercept = new Translation2d();
        Translation2d output = new Translation2d();

        double slope = y / x;

        if (slope == 0 || Double.isNaN(slope)) {

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
                        (Math.pow(x1, 2) + Math.pow(y1, 2))
                                /
                                (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

}
