package debug;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Debug {

  public static GenericEntry xP; public static GenericEntry xD; 
  public static GenericEntry yP; public static GenericEntry yD; 
  public static GenericEntry rotP; public static GenericEntry rotD; 
  public static GenericEntry xDiff; public static GenericEntry yDiff; public static GenericEntry rotDiff;
  public static GenericEntry xPos; public static GenericEntry yPos;

  public void debugInit() {

    xP = Shuffleboard.getTab("Drive")
      .add("xP", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 10)) // specify widget properties here
      .getEntry();

    xD = Shuffleboard.getTab("Drive")
      .add("xD", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    yP = Shuffleboard.getTab("Drive")
    .add("yP", 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 10)) // specify widget properties here
    .getEntry();

    yD = Shuffleboard.getTab("Drive")
      .add("yD", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();
   
    rotP = Shuffleboard.getTab("Turn")
      .add("rotP", 0.22)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -5, "max", 5)) // specify widget properties here
      .getEntry();

    rotD = Shuffleboard.getTab("Turn")
      .add("rotD", 0.74)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    xDiff = Shuffleboard.getTab("Drive")
      .add("xDiff", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
      .getEntry();

    yDiff = Shuffleboard.getTab("Drive")
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

}
