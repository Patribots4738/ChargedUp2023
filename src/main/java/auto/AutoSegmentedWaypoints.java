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
import io.github.oblarg.oblog.annotations.Log.Logs;
import calc.Constants.AlignmentConstants;
import calc.Constants.PlacementConstants;

public class AutoSegmentedWaypoints implements Loggable {

  Swerve swerve;
  Arm arm;
  Claw claw;
  AutoAlignment autoAlignment;

  public Waypoint[] chosenWaypoints;
  public AutoPose chosenAutoPath;

  @Log
  public int currentWaypointNumber = 0;

  @Log
  public double autoDelay;

  @Log
  public boolean stateHasFinished = false;
  @Log
  public boolean stateHasInitialized = false;
  @Log
  public boolean clawHasStarted = false;
  @Log
  public boolean hasMovedArm = false;
  @Log
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

    chosenWaypoints = chosenAutoPath.getWaypointSet();

    currentWaypointNumber = 0;

    PathPlannerState initialPathPose = chosenWaypoints[0].getPathPlannerSegment().getInitialState();

    // Pose2d mirroredPose = new Pose2d(
    //   (((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? AlignmentConstants.FIELD_WIDTH_METERS : 0) + (initialPathPose.poseMeters.getTranslation().getX() * ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? -1 : 1))), 
    //     initialPathPose.poseMeters.getTranslation().getY(), 
    //     initialPathPose.holonomicRotation.minus(Rotation2d.fromRadians((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? Math.PI : 0)));
    
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red && DriverStation.isAutonomous()) {

      initialPathPose.poseMeters = new Pose2d(
        (AlignmentConstants.FIELD_WIDTH_METERS - initialPathPose.poseMeters.getTranslation().getX()), 
        initialPathPose.poseMeters.getTranslation().getY(), 
        initialPathPose.poseMeters.getRotation().unaryMinus().plus(Rotation2d.fromDegrees(Math.PI)));

      initialPathPose.holonomicRotation = initialPathPose.poseMeters.getRotation().plus(Rotation2d.fromRadians(Math.PI)).unaryMinus();

    }

    stateHasFinished = false;
    stateHasInitialized = false;
    clawHasStarted = false;
    hasMovedArm = false;
    startedChargePad = false;

    swerve.resetOdometry(new Pose2d(initialPathPose.poseMeters.getTranslation(), initialPathPose.holonomicRotation));

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

    if (clawSpeed == PlacementConstants.CLAW_OUTTAKE_SPEED && !clawHasStarted) {
      claw.setDesiredSpeed(PlacementConstants.CLAW_INTAKE_SPEED);
    }

    // Check if the arm is ready to move to the next waypoint mid-path
    if (SwerveTrajectory.trajectoryStatus.equals("execute") && currentWaypointNumber != 0 && arm.getAtDesiredPositions()) {

      // Prepare the arm for the next waypoint before the path is done
      switch (armIndex) {
        // If the next waypoint is a floor pickup, prepare the arm for a floor pickup
        case PlacementConstants.CUBE_INTAKE_INDEX:
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
    else if (SwerveTrajectory.trajectoryStatus.equals("done")) {
      // If the arm is not in the desired position, move it
      if (!hasMovedArm) {
        // If the arm is at a prep index, change the desired index to be the other half of that prep index...
        // Notice that the floor intake does not have a second half, 
        // this is because the first transition point is the prep index's end point
        if (arm.getArmIndex() == PlacementConstants.HIGH_CONE_PREP_INDEX)
        {
          System.out.println("Moving arm to index HIGH_PLACE_AUTO (" + PlacementConstants.HIGH_PLACE_INDEX_AUTO + ") at waypoint: " + currentWaypointNumber);
          arm.setArmIndex(PlacementConstants.HIGH_PLACE_INDEX_AUTO);
        }
        // The arm is not at a prep index...
        else {
          System.out.println("Moving arm to index: " + armIndex + " at waypoint: " + currentWaypointNumber);
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
      if ((Timer.getFPGATimestamp() - autoDelay > 0.3) || (clawSpeed == PlacementConstants.CLAW_STOPPED_SPEED)) {
        stateHasFinished = true;
      }
    }
  }

  public void waypointRunner(Waypoint[] thisWaypointSet) {
    // If we made one round with the state, we have successfully initialized
    if (!stateHasInitialized) {
      SwerveTrajectory.resetTrajectoryStatus();
      stateHasInitialized = true;
    }

    // Only run the path if the robot isn't trying to align
    // This will give priority to the alignment
    if (!startedChargePad) {
      SwerveTrajectory.PathPlannerRunner(thisWaypointSet[currentWaypointNumber].getPathPlannerSegment(), swerve);
    }
    this.setArmIndex(thisWaypointSet[currentWaypointNumber].getArmPosIndex(), thisWaypointSet[currentWaypointNumber].getClawDirection());

    // If there are not more waypoints, tell the robot to level itself
    // We can do this if we want to go on chargepad or not
    // becuase if we did not want to then the robot is already leveled.
    if ((currentWaypointNumber == chosenWaypoints.length - 1) && 
        Math.abs(swerve.getPitch().getDegrees()) > 7 || Math.abs(swerve.getRoll().getDegrees()) > 7)
    {
      System.out.println("Charging!");
      if (!startedChargePad) {
        // Set the timer for the charge pad leveing PID loop
        autoAlignment.startChargePad();
        // Prevent startChargePad from being called again
        startedChargePad = true;
      }
      // Run the charge pad leveling PID loop
      autoAlignment.chargeAlign();
    }
  
    if (stateHasFinished) {

      if (arm.getArmIndex() == PlacementConstants.HIGH_CONE_PLACEMENT_INDEX || 
          arm.getArmIndex() == PlacementConstants.HIGH_PLACE_INDEX_AUTO) 
      { 
        arm.setArmIndex(PlacementConstants.HIGH_TO_STOWWED_INDEX); 
      }
      else { 
        arm.setArmIndex(PlacementConstants.STOWED_INDEX); 
      }

      if (currentWaypointNumber < chosenWaypoints.length - 1) {
        currentWaypointNumber++;
        stateHasInitialized = false;
      }
      
      // Only move the claw before the arm
      // if it needs to hold a game piece
      if (thisWaypointSet[currentWaypointNumber].getClawDirection() != PlacementConstants.CLAW_OUTTAKE_SPEED)
      {
        claw.stopClaw();
      }

      stateHasFinished = false;
      clawHasStarted = false;
      hasMovedArm = false;
    }
  }
}