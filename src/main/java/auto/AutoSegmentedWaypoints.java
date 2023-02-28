// Referenced from https://github.com/Stampede3630/2022-Code/blob/0ad2aa434f50d8f5dc93e965809255f697dadffe/src/main/java/frc/robot/AutoSegmentedWaypoints.java#L81
package auto;

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
import calc.Constants.AlignmentConstants;
import calc.Constants.PlacementConstants;

public class AutoSegmentedWaypoints implements Loggable {

  Swerve swerve;
  Arm arm;
  Claw claw;
  AutoAlignment autoAlignment;
  AutoPathStorage autoPathStorage;
  
  public Waypoint[] chosenWaypoints;
  public AutoPose chosenAutoPath;

  public int currentWaypointNumber = 0;

  @Log
  public double autoDelay;

  public boolean stateHasFinished = false;
  public boolean stateHasInitialized = false;
  public boolean clawHasStarted = false;
  public boolean hasMovedArm = false;
  public boolean startedChargePad = false;

  public AutoSegmentedWaypoints(Swerve swerve, Arm arm, Claw claw, AutoAlignment autoAlignment) {
    this.swerve = swerve;
    this.arm = arm;
    this.claw = claw;
    this.autoAlignment = autoAlignment;
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
    
        
    stateHasFinished = false;
    stateHasInitialized = false;
    clawHasStarted = false;
    hasMovedArm = false;
    startedChargePad = false;
    
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

    // Check if the arm is ready to move to the next waypoint mid-path
    if (SwerveTrajectory.trajectoryStatus.equals("execute") && currentWaypointNumber != 0 && arm.getAtDesiredPositions()) {

      // Prepare the arm for the next waypoint before the path is done
      switch (armIndex) {
        // If the next waypoint is a floor pickup, prepare the arm for a floor pickup
        case PlacementConstants.FLOOR_INTAKE_INDEX:
          arm.setArmIndex(PlacementConstants.FLOOR_INTAKE_PREP_INDEX);
          break;

        // If the next waypoint is a high cone placement, prepare the arm for a high cone placement
        case PlacementConstants.HIGH_CONE_PLACEMENT_INDEX:
          arm.setArmIndex(PlacementConstants.HIGH_CONE_PREP_INDEX);
          break;
      }
      // Do not change hasMovedArm because we are not done moving the arm
    }

    // Once the robot is in position...
    if (SwerveTrajectory.trajectoryStatus.equals("done")) {
      // If the arm is not in the desired position, move it
      if (!hasMovedArm) {
        // If the arm is at a prep index, change the desired index to be the other half of that prep index...
        // Notice that the floor intake does not have a second half, 
        // this is because the first transition point is the prep index's end point
        if (arm.getArmIndex() == PlacementConstants.HIGH_CONE_PREP_INDEX)
        {
          arm.setArmIndex(PlacementConstants.HIGH_PLACE_INDEX_AUTO);
        }
        // The arm is not at a prep index...
        else {
          arm.setArmIndex(armIndex);
        }
        // Prevent the arm from setting the index again
        hasMovedArm = true;
      }
    }

    // Once the arm and robot are in position...
    if ((SwerveTrajectory.trajectoryStatus.equals("done") && arm.getAtDesiredPositions())) {

      // Move the claw to the desired speed
      if (!clawHasStarted && (clawSpeed != PlacementConstants.CLAW_STOPPED_SPEED)) { 
        // Set autoDelay to the current time,
        // This is to allow the claw to move for a bit before moving on.
        autoDelay = Timer.getFPGATimestamp();
        claw.setDesiredSpeed(clawSpeed);
        // Prevent the autoDelay from being reset
        clawHasStarted = true;
      }
      // 0.3 seconds since the claw has moved (and if there are more waypoints)
      if ((Timer.getFPGATimestamp() - autoDelay > 0.3) && (currentWaypointNumber < chosenWaypoints.length - 1)) {
        stateHasFinished = true;
      }
      // If there are not more waypoints, tell the robot to level itself
      // We can do this if we want to go on chargepad or not
      // becuase if we did not want to then the robot is already leveled.
      else {
        if (!startedChargePad) {
          // Set the timer for the charge pad leveing PID loop
          autoAlignment.startChargePad(); 
          // Prevent startChargePad from being called again
          startedChargePad = true;
        }
        // Run the charge pad leveling PID loop
        autoAlignment.chargeAlign();
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

      if (arm.getArmIndex() == PlacementConstants.HIGH_CONE_PLACEMENT_INDEX || 
          arm.getArmIndex() == PlacementConstants.HIGH_PLACE_INDEX_AUTO) { arm.setArmIndex(PlacementConstants.HIGH_TO_STOWWED_INDEX); }
      else { arm.setArmIndex(PlacementConstants.STOWED_INDEX); }

      // Only move the claw before the arm
      // if it needs to hold a game piece
      if ((claw.getDesiredSpeed() != PlacementConstants.CLAW_INTAKE_SPEED &&
      claw.getDesiredSpeed() != PlacementConstants.CLAW_INTAKE_SPEED)) {
        claw.stopClaw();
      }
      
      currentWaypointNumber++;

      stateHasFinished = false;
      stateHasInitialized = false;
      clawHasStarted = false;
      hasMovedArm = false;
    }
  }
}