package auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Auto {
  
  int autoStates;
  /**
   * Returns a new HolonomicDriveController with the given PID gains.
   * 
   * The 2 PID controllers are controllers that should correct 
   * for error in the field-relative x and y directions respectively.
   * i.e. xP,I,D is PID for X correction
   *  and yP,I,D is PID for Y correction
   * 
   * The ProfiledPIDController for the rotation of the robot, 
   * utilizing a Trapezoidprofile for smooth locomotion in terms of max velocity and acceleration
   * 
   * @param xP The x-axis' P gain for the PID controller
   * @param xI The x-axis' I gain for the PID controller
   * @param xD The x-axis' D gain for the PID controller
   * @param yP The y-axis' P gain for the PID controller
   * @param yI The y-axis' I gain for the PID controller
   * @param yD The y-axis' D gain for the PID controller
   * @param rotP The rotational-axis' P gain for the PID controller
   * @param rotI The rotational-axis' I gain for the PID controller
   * @param rotD The rotational-axis' D gain for the PID controller
   * @param maxVel The maximum velocity that the robot can TURN in the trapezoidal profile
   * @param maxAccel The maximum acceleration that the robot can TURN in the trapezoidal profile
   * @return A new HolonomicDriveController with the given PID gains (xP, xI, xD, yP, yI, yD, rotP, rotI, rotD) and constraints (maxVel, maxAccel)
   */
  public HolonomicDriveController getAutoController(double xP, double xI, double xD,
                                                      double yP, double yI, double yD,
                                                      double rotP, double rotI, double rotD,
                                                        double maxVel, double maxAccel) {

    return new HolonomicDriveController(
        new PIDController(xP, xI, xD),
        new PIDController(yP, yI, yD),
        new ProfiledPIDController(rotP, rotI, rotD,
          new TrapezoidProfile.Constraints(maxVel, maxAccel)));

  }

  public int getAutoStates() {
    return this.autoStates;
  }
}
