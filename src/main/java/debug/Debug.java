package debug;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import math.Constants.AlignmentConstants;
import math.Constants.AutoConstants;

import java.util.Map;

public class Debug {

    public static GenericEntry xP;
    public static GenericEntry xD;
    public static GenericEntry yP;
    public static GenericEntry yD;
    public static GenericEntry rotP;
    public static GenericEntry rotD;
    public static GenericEntry xDiff;
    public static GenericEntry yDiff;
    public static GenericEntry rotDiff;
    public static GenericEntry xPos;
    public static GenericEntry yPos;
    public static GenericEntry maxAngularSpeed;
    public static GenericEntry maxAngularAccel;

  public Debug() {

    xP = Shuffleboard.getTab("Drive")
        .add("xP", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
        .getEntry();

    xD = Shuffleboard.getTab("Drive")
        .add("xD", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
        .getEntry();

    yP = Shuffleboard.getTab("Drive")
        .add("yP", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
        .getEntry();

    yD = Shuffleboard.getTab("Drive")
        .add("yD", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
        .getEntry();

    rotP = Shuffleboard.getTab("Turn")
        .add("rotP", AutoConstants.ROTATION_CORRECTION_D)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
        .getEntry();

    rotD = Shuffleboard.getTab("Turn")
        .add("rotD", AutoConstants.ROTATION_CORRECTION_D)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
        .getEntry();

    xDiff = Shuffleboard.getTab("Drive")
        .add("xDiff", 0)
        .withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();

    yDiff = Shuffleboard.getTab("Dri-1")
        .add("yDiff", 0)
        .withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();

    rotDiff = Shuffleboard.getTab("Turn")
        .add("rotDiff", 0)
        .withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();

    xPos = Shuffleboard.getTab("Arm")
        .add("xPos", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -Math.PI, "max", Math.PI)) // specify widget properties here
        .getEntry();

    yPos = Shuffleboard.getTab("Arm")
        .add("yPos", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -Math.PI, "max", Math.PI)) // specify widget properties here
        .getEntry();
  }


  public static void debugPeriodic(double x, double y, double rot) {

    xDiff.setDouble(x);
    yDiff.setDouble(y);
    rotDiff.setDouble(rot);

  }

  public static void printArmAngles(Translation2d armInputs, double upperAngle, double lowerAngle) {
    System.out.println(
        "LeftX: " + String.format("%.3f", armInputs.getX()) +
            " LeftY: " + String.format("%.3f", armInputs.getY()) +
            " Q1: " + String.format("%.3f", Units.radiansToDegrees(lowerAngle)) +
            " Q2: " + String.format("%.3f", Units.radiansToDegrees(upperAngle) - 90));
  }
  

}
