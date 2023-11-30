package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Constants.FieldConstants.GameMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private Command disabledCommand;

    private RobotContainer robotContainer;


    @Override
    public void robotInit() { 
        robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Used for items like diagnostics
     * ran during disabled, autonomous, teleoperated and test. :D
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        disabledCommand = robotContainer.getDisabledCommand();
        
        if (disabledCommand != null) disabledCommand.schedule();
    }

    @Override
    public void disabledPeriodic() { }

    @Override
    public void autonomousInit() { 

        robotContainer.onEnabled(GameMode.AUTONOMOUS);

        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null) autonomousCommand.schedule();

        if (disabledCommand != null) disabledCommand.cancel();
    }

    @Override
    public void autonomousPeriodic() { }
    
    @Override
    public void teleopInit() {
        robotContainer.onEnabled(GameMode.TELEOP);
        // Stop our autonomous command if it is still running.
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        if (disabledCommand != null) disabledCommand.cancel();
    }

    @Override
    public void teleopPeriodic() { }
    
    @Override
    public void testInit() { 
        // Cancels all running commands at the start of test mode.
        robotContainer.onEnabled(GameMode.TEST);
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        if (disabledCommand != null) disabledCommand.cancel();
    }

    @Override
    public void testPeriodic() { }

    @Override
    public void simulationInit() { }

    @Override
    public void simulationPeriodic() { 
        REVPhysicsSim.getInstance().run();
    }
}
