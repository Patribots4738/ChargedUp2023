# ChargedUp 2023
![Gradle Build](https://img.shields.io/github/actions/workflow/status/Patribots4738/ChargedUp2023/gradle.yml?label=Gradle%20Build&logo=Gradle) | [`src/main/java`](src/main/java/) shortcut
 
![Robot Image](images/Jerome.gif)
____
![ChargedUp-2023](https://upload.wikimedia.org/wikipedia/en/thumb/b/b7/Charged_Up_Logo.svg/220px-Charged_Up_Logo.svg.png)

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 

This repository is entirely student-created and maintained.
We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, and be sure to star this repo if you found anything helpful!

### [See how we did!](https://www.statbotics.io/team/4738)

***[Check out our release video!](https://www.youtube.com/watch?v=b3fmnyfDZ1o)***

___

## Highlights
  - Field-centric swerve drive 
  - Two-link arm driven by inverse kinematics
  - Modular autonomous routines
  - Auto balance for Charge Pad using gyroscope
  - Auto alignment and placement for cubes and cones
  - April Tag interpretation using PhotonVision

## Major Package Functions

  - [`src.main.java.calc`](src/main/java/calc)

    - Contains all calculations done for inverse kinematic equations in [`ArmCalculations.java`](src/main/java/calc/ArmCalculations.java), AprilTag interpretation in [`PhotonCameraUtil.java`](src/main/java/calc/PhotonCameraUtil.java), and custom joystick math in [`OICalc.java`](src/main/java/calc/OICalc.java).
    
    Don't forget about [Constants](src/main/java/calc/Constants.java)!

  - [`src.math.java.hardware`](src/main/java/hardware)
	
    - The source of our subsystems, where motors are driven. This include the [Arm](src/main/java/hardware/Arm.java), [Claw](src/main/java/hardware/Claw.java), and [Swerve](src/main/java/hardware/Swerve.java) drive (which is made up of [MAXSwerveModules](src/main/java/hardware/MAXSwerveModule.java)).
    - You can also find our [LEDController](src/main/java/hardware/LEDController/LEDController.ino) and [calibration](src/main/java/hardware/LEDCallibration/LEDCallibration.ino) tool, which communicates with an Arduino Uno for our [LEDs](src/main/java/hardware/LEDController/LEDController.ino).

Want to find out more?
[`Jump into the code!`](src/main/java/)

____
# ***Controls, courtesy of 3255 <3***
![Driver Controller](images/driverController.png)
![Operator Controller](images/operatorController.png)
