package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;

public class PatriBoxController extends CommandXboxController {

    private double deadband;

    public PatriBoxController(int port, double deadband) {
        super(port);

        this.deadband = deadband;
    }

    @Override
    public double getLeftX() {
        return getLeftAxis().getX();
    }

    @Override
    public double getLeftY() {
        return getLeftAxis().getY();
    }

    public Translation2d getLeftAxis() {
        Translation2d driverLeftAxis = toCircle(MathUtil.applyDeadband(super.getLeftX(), deadband),
                MathUtil.applyDeadband(super.getLeftY(), deadband));

        if (FieldConstants.ALLIANCE == Alliance.Blue) {
            driverLeftAxis = driverLeftAxis.unaryMinus();
        }

        return driverLeftAxis;
    }

    @Override
    public double getRightX() {
        return getRightAxis().getX();
    }

    @Override
    public double getRightY() {
        return getRightAxis().getY();
    }

    public Translation2d getRightAxis() {
        return toCircle(MathUtil.applyDeadband(super.getRightX(), deadband),
                MathUtil.applyDeadband(super.getRightY(), deadband));
    }

    // All calculations can be referenced here
    // https://www.desmos.com/calculator/e07raajzh5
    private Translation2d toCircle(double x, double y) {

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
        return Math.sqrt(
                (Math.pow(x1, 2) + Math.pow(y1, 2)) /
                        (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

}
