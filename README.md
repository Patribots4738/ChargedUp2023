# ChargedUp 2023
![Gradle Build](https://img.shields.io/github/actions/workflow/status/Patribots4738/ChargedUp2023/gradle.yml?label=Gradle%20Build&logo=Gradle)


![Robot Image](src\main\deploy\images\jerome.jpg)

## Highlights
  - Field-Relative Swerve Drive 
  - Inc

## Major Package Functions

  - [`src.main.java.calc`](src\main\java\calc)

    Contains all calculations done to for reverse kinematic equations, Photon Vision, etc...
    - [`ArmCalculations.java`](src\main\java\calc\ArmCalculations.java)
      - Contains reverse kinematic equations to find the angle at which the arm should be at to reach a desired position. 

    - [`PhotonCameraUtil.java`]()
      - Contains constructor and other getter and setter methods to calculate the robots position, given the coordinates from the photon camera. 

  - [`Swerve.java`](src\main\java\hardware\Swerve.java)

    Want to find out more about how we do swerve? Look here :D

  - [`LEDController.ino`](src\main\java\hardware\LEDController\LEDController.ino)

    We use LED strips on our robot! Here is our LED code for those lights 

    - [`ArduinoController.java`](src\main\java\hardware\ArduinoController.java)

Want to find out more ?
[`Jump into the code!`](src/main/java/)

____

![ChargedUp-2023](https://upload.wikimedia.org/wikipedia/en/thumb/b/b7/Charged_Up_Logo.svg/220px-Charged_Up_Logo.svg.png)

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 

This repository is entirely student-created and maintained.
We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, and be sure to star this repo if you found anything helpful!


- [`com.team1678.frc2023`](src/main/java/com/team1678/frc2023/)
	
	Contains central robot functions specific to Tangerine Tumbler.  Robot control originates from the [`Robot`](/src/main/java/com/team1678/frc2023/Robot.java) class.

- [`com.team1678.frc2023.auto`](src/main/java/com/team1678/frc2023/auto)
	
	Handles generation, selection, and execution of autonomous routines.

- [`com.team1678.frc2023.auto.actions`](src/main/java/com/team1678/frc2023/auto/actions/)
	
	Contains all actions used during autonomous.  All actions must implement the [`Action`](src/main/java/com/team1678/frc2023/auto/actions/Action.java) interface.  

- [`com.team1678.frc2023.auto.modes`](src/main/java/com/team1678/frc2023/auto/modes/)
	
	Contains all autonomous modes.  Modes are named according to starting location (CC for cable chain side, middle for center, nothing for flat side), number of game pieces scored (One/Two/Three), and if it engages.

 - [`com.team1678.frc2023.controlboard`](src/main/java/com/team1678/frc2023/controlboard/)
	
	Handles polling driver and operator inputs from two [`Xbox Controllers`](src/main/java/com/team1678/frc2023/controlboard/CustomXboxController.java).

 - [`com.team1678.frc2023.loops`](src/main/java/com/team1678/frc2023/loops/)
	
	Contains code for loops, which are run periodically to update Subsystems.  [`Loops`](src/main/java/com/team1678/frc2023/loops/Loop.java) are managed and run by [`Loopers`](src/main/java/com/team1678/frc2023/loops/Looper.java).  The [`Robot`](/src/main/java/com/team1678/frc2023/Robot.java) class contains three loops: one for enabled operation, one for IO while disabled, and one for CSV logging.

- [`com.team1678.frc2023.shuffleboard`](src/main/java/com/team1678/frc2023/shuffleboard/)
	
	Contains layouts for reporting telemetry to the Shuffleboard.  Entries are organized into [`Tabs`](src/main/java/com/team1678/frc2023/shuffleboard/tabs/), which extend the [`ShuffleboardTabBase`](src/main/java/com/team1678/frc2023/shuffleboard/ShuffleboardTabBase.java).

- [`com.team1678.frc2023.states`](src/main/java/com/team1678/frc2023/states/)

	Contains one class, [`SuperstructureGoal`](src/main/java/com/team1678/frc2023/states/SuperstructureGoal.java), which holds preset setpoints for the arm, elevator, and wrist.

- [`com.team1678.frc2023.subsystems`](src/main/java/com/team1678/frc2023/subsystems/)

	Contains code for subsystems, with one singleton implementation per subsystem.  Subsystems extend the [`Subsystem`](src/main/java/com/team1678/frc2023/subsystems/Subsystem.java) abstract class. Each subsystem's logic is contained in an enabled loop, a read periodic inputs method, and a write periodic outputs method, which are called by the [`SubsystemManager`](src/main/java/com/team1678/frc2023/SubsystemManager.java) class.  


- [`com.team1678.lib.logger`](src/main/java/com/team1678/lib/logger/)

	Contains code for logging fields tagged by the [`Log`](src/main/java/com/team1678/lib/logger/Log.java) annotation to CSV files.

