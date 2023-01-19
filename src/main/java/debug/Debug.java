package debug;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Debug {

  GenericEntry kP; GenericEntry kI; GenericEntry kD; GenericEntry kP2; GenericEntry kI2; GenericEntry kD2; GenericEntry m_speed;

  public void debugInit() {
    kP = Shuffleboard.getTab("Drive")
      .add("P", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 10)) // specify widget properties here
      .getEntry();

    kI = Shuffleboard.getTab("Drive")
      .add("I", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    kD = Shuffleboard.getTab("Drive")
      .add("D", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();
   
    kP2 = Shuffleboard.getTab("Turn")
      .add("P", 0.22)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -5, "max", 5)) // specify widget properties here
      .getEntry();

    kI2 = Shuffleboard.getTab("Turn")
      .add("I", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    kD2 = Shuffleboard.getTab("Turn")
      .add("D", 0.74)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", -1, "max", 1)) // specify widget properties here
      .getEntry();

    m_speed = Shuffleboard.getTab("Turn")
      .add("Speed", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
      .getEntry();
  }

  public void debugPeriodic(double vYspeed) {
    m_speed.setDouble(vYspeed);
  }

}
