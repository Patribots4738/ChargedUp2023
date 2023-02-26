// Referenced from https://github.com/Stampede3630/2022-Code/blob/0ad2aa434f50d8f5dc93e965809255f697dadffe/src/main/java/frc/robot/AutoSegmentedWaypoints.java#L81
package auto;

import java.sql.Driver;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import auto.AutoPathStorage.AutoPose;
import auto.AutoPathStorage.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import hardware.Arm;
import hardware.Claw;
import hardware.Swerve;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import math.Constants.AlignmentConstants;
import math.Constants.AutoConstants;
import math.Constants.PlacementConstants;

public class AutoSegmentedWaypoints implements Loggable {

  Swerve swerve;
  Arm arm;
  Claw claw;
  AutoPathStorage autoPathStorage;
  
  public Waypoint[] chosenWaypoints;
  public AutoPose chosenAutoPath;

  public int currentWaypointNumber = 0;

  @Log
  public double autoDelay;

  public boolean stateHasFinished = false;
  public boolean stateHasInitialized = false;
  public boolean clawHasStarted = false;

  public AutoSegmentedWaypoints(Swerve swerve, Arm arm, Claw claw) {
    this.swerve = swerve;
    this.arm = arm;
    this.claw = claw;
  }

  public void init() {

    if (AutoPathStorage.autoChooser.getSelected() == null) {

      chosenAutoPath = AutoPathStorage.myAutoContainer[0];

    } else {

      chosenAutoPath = AutoPathStorage.autoChooser.getSelected();

    }

    chosenWaypoints = chosenAutoPath.thisWPset;

    currentWaypointNumber = 0;

    PathPlannerState initialPathPose = chosenWaypoints[0].pathPlannerSegment.getInitialState();
    chosenWaypoints[0].pathPlannerSegment.getInitialHolonomicPose();

    Pose2d mirroredPose = new Pose2d(
      (((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? AlignmentConstants.FIELD_WIDTH_METERS : 0) + (initialPathPose.poseMeters.getTranslation().getX() * ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? -1 : 1))), 
        initialPathPose.poseMeters.getTranslation().getY(), 
        initialPathPose.holonomicRotation.plus(Rotation2d.fromRadians((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? Math.PI : 0)));
    
    System.out.println(mirroredPose);

    swerve.resetOdometry(mirroredPose);

  }

  public void periodic() {
    waypointRunner(chosenWaypoints);
  }

  /**
   * This method moves both arms to a position
   * <p>
   * If both arms are in the set position, then
   * stop the auto path and move on to the next waypoint
   */
  private void setArmIndex(int armIndex, double clawSpeed) {

    // Only move the claw before the arm
    // if it needs to hold a game piece
    if (claw.getDesiredSpeed() != PlacementConstants.CLAW_CONE_INTAKE_SPEED &&
        claw.getDesiredSpeed() != PlacementConstants.CLAW_CUBE_INTAKE_SPEED) {
      claw.stopClaw();
    }

    if (SwerveTrajectory.trajectoryStatus.equals("done")) {

      arm.setArmIndex(armIndex);

    } else {

      autoDelay = Timer.getFPGATimestamp();

    }
    
    if (SwerveTrajectory.trajectoryStatus.equals("done") && arm.getAtDesiredPositions()) {

      if (!clawHasStarted) { 
        autoDelay = Timer.getFPGATimestamp();
        clawHasStarted = true;
      }
      claw.setDesiredSpeed(clawSpeed);
      // 1.5 seconds since the path has completed
      if (Timer.getFPGATimestamp() - autoDelay > 1) {
        
        if (currentWaypointNumber < chosenWaypoints.length - 1) {
          stateHasFinished = true;
        }

      }
    }
  }

  public void waypointRunner(Waypoint[] thisWaypointSet) {
    // If we made one round with the state, we have successfully initialized
    if (!stateHasInitialized) {
      SwerveTrajectory.resetTrajectoryStatus();
      stateHasInitialized = true;
    }

    SwerveTrajectory.PathPlannerRunner(thisWaypointSet[currentWaypointNumber].pathPlannerSegment, swerve);

    this.setArmIndex(thisWaypointSet[currentWaypointNumber].armPosIndex, thisWaypointSet[currentWaypointNumber].clawDirection);

    if (stateHasFinished) {

      arm.setArmIndex(PlacementConstants.STOWED_INDEX);
      
      currentWaypointNumber++;

      stateHasFinished = false;
      stateHasInitialized = false;
      clawHasStarted = false;
    }
  }
}