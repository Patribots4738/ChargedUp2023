package auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import math.Constants;

public class Auto {
  
  int autoStates;
  /**
   * Returns a new HolonomicDriveController with constant PID gains.
   * 
   * The 2 PID controllers are controllers that should correct 
   * for error in the field-relative x and y directions respectively.
   * i.e. kXCorrectionP,I,D is PID for X correction
   *  and kYCorrectionP,I,D is PID for Y correction
   * 
   * The ProfiledPIDController for the rotation of the robot, 
   * utilizing a Trapezoidprofile for smooth locomotion in terms of max velocity and acceleration
   * 
   * @see kXCorrectionP The x-axis' P gain for the PID controller
   * @see kXCorrectionI The x-axis' I gain for the PID controller
   * @see kXCorrectionD The x-axis' D gain for the PID controller
   * @see kYCorrectionP The y-axis' P gain for the PID controller
   * @see kYCorrectionI The y-axis' I gain for the PID controller
   * @see kYCorrectionD The y-axis' D gain for the PID controller
   * @see kRotationCorrectionP The rotational-axis' P gain for the PID controller
   * @see kRotationCorrectionI The rotational-axis' I gain for the PID controller
   * @see kRotationCorrectionD The rotational-axis' D gain for the PID controller
   * @see kMaxAngularSpeedRadiansPerSecond The maximum velocity that the robot can TURN in the trapezoidal profile
   * @see kMaxAngularSpeedRadiansPerSecondSquared The maximum acceleration that the robot can TURN in the trapezoidal profile
   * @return A new HolonomicDriveController with the given PID gains (xP, xI, xD, yP, yI, yD, rotP, rotI, rotD) and constraints (maxVel, maxAccel)
   */
  public HolonomicDriveController getAutoController() {
    return new HolonomicDriveController(
      new PIDController(Constants.AutoConstants.kXCorrectionP, Constants.AutoConstants.kXCorrectionI, Constants.AutoConstants.kXCorrectionD),
      new PIDController(Constants.AutoConstants.kYCorrectionP, Constants.AutoConstants.kYCorrectionI, Constants.AutoConstants.kYCorrectionD),
      new ProfiledPIDController(Constants.AutoConstants.kRotationCorrectionP, Constants.AutoConstants.kRotationCorrectionI, Constants.AutoConstants.kRotationCorrectionD,
        new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)));

  }

  public int getAutoStates() {
    return this.autoStates;
  }
}
