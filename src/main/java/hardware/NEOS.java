// Developed by Reza from Team Spyder 1622

package hardware;

import com.revrobotics.*;

import calc.Constants.NeoMotorConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;

/*
 * Some of this is adapted from 3005's 2022 Code
 * Original source published at https://github.com/FRC3005/Rapid-React-2022-Public/tree/d499655448ed592c85f9cfbbd78336d8841f46e2
 */

public class NEOS extends CANSparkMax {
    public final RelativeEncoder encoder;
    public final SparkMaxPIDController pidController;

    private List<NEOS> followers = new ArrayList<>();

    private ControlLoopType controlType = ControlLoopType.PERCENT;
    private double targetPosition = 0;
    private double targetVelocity = 0;
    
    /**
     * Creates a new NEOS motor
     * @param id CANID of the SparkMax the NEOS is connected to.
     */
    public NEOS(int id) {
        this(id, false);
    }

    /**
     * Creates a new NEOS motor
     * @param id CANID of the SparkMax the NEOS is connected to.
     * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If false, the motor will coast when not powered.
     */
    public NEOS(int id, CANSparkMax.IdleMode mode) {
        this(id, false, mode);
    }

    /**
     * Creates a new NEOS motor
     * @param id CANID of the SparkMax the NEOS is connected to.
     * @param reversed Whether the motor is reversed or not.
     * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If false, the motor will coast when not powered.
     */
    public NEOS(int id, boolean reversed, CANSparkMax.IdleMode mode) {
        super(id, CANSparkMaxLowLevel.MotorType.kBrushless);

        // restoreFactoryDefaults();
        // Timer.delay(0.050);

        // If a parameter set fails, this will add more time to alleviate any bus traffic
        // default is 20ms
        setCANTimeout(50);

        register();

        encoder = getEncoder();
        pidController = getPIDController();
    }

    /**
     * Creates a new NEOS motor
     * @param id CANID of the SparkMax the NEOS is connected to.
     * @param reversed Whether the motor is reversed or not.
     */
    public NEOS(int id, boolean reversed) {
        this(id, reversed, CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Sets the target position for the NEO.
     * @param position Position to set the NEOS to in rotations.
     * @param arbitraryFeedForward Arbitrary feed forward to add to the motor output.
     */
    public synchronized void setTargetPosition(double position, double arbitraryFeedForward, int slot) {
        if (Robot.isReal()) {
            pidController.setReference(position, ControlType.kPosition, slot, arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
        }
        targetPosition = position;
        controlType = ControlLoopType.POSITION;
    }

    public synchronized void setTargetPosition(double position, double arbitraryFeedForward) {
        setTargetPosition(position, arbitraryFeedForward, 0);
    }

    public synchronized void setTargetPosition(double position) {
        setTargetPosition(position, 0, 0);
    }

    /**
     * Sets the target velocity for the NEO.
     * @param velocity Velocity to set the NEOS to in rotations per minute.
     */
    public synchronized void setTargetVelocity(double velocity) {
        setTargetVelocity(velocity, 0, 0);
    }

    /**
     * Sets the target velocity for the NEO.
     * @param velocity Velocity to set the NEOS to in rotations per minute.
     * @param arbitraryFeedForward Arbitrary feed forward to add to the motor output.
     */
    public synchronized void setTargetVelocity(double velocity, double arbitraryFeedForward, int slot) {
        if (velocity == 0) {
            setVoltage(0);
        } else {
            pidController.setReference(velocity, ControlType.kVelocity);
        }
        targetVelocity = velocity;
        controlType = ControlLoopType.VELOCITY;
    }

    public synchronized void set(double percent) {
        setVoltage(percent * RobotController.getBatteryVoltage());
        controlType = ControlLoopType.PERCENT;
    }

    private boolean shouldCache = false;
    private double position = 0;
    private double velo = 0;

    public void tick() {
        if (shouldCache) {
            position = encoder.getPosition();
            velo = encoder.getVelocity();
        }

        if (Robot.isSimulation() && controlType == ControlLoopType.POSITION) {
            setVoltage(pidController.getP() * (targetPosition - getPosition()));
        }
    }

    public void register() {
        NeoMotorConstants.motors.add(this);
        REVPhysicsSim.getInstance().addSparkMax(this, DCMotor.getNEO(1));
    }

    /**
     * Gets the position of the NEOS in rotations.
     * @return The position of the NEOS in rotations relative to the last 0 position.
     */
    public double getPosition() {
        double pos;
        if (shouldCache) {
            pos = position;
        } else {
            pos = encoder.getPosition();
        }

        if (Robot.isSimulation() && controlType == ControlLoopType.VELOCITY) {
            pos /= encoder.getVelocityConversionFactor();
        }

        return pos;
    }

    /**
     * Gets the velocity of the NEOS in rotations per minute.
     * @return The instantaneous velocity of the NEOS in rotations per minute.
     */
    public double getVelocity() {
        if (shouldCache) {
            return velo;
        } else {
            return encoder.getVelocity();
        }
    }

    public synchronized void setPosition(double position) {
        encoder.setPosition(position);
    }

    /**
     * Gets the target position of the NEOS in rotations.
     * @return The target position of the NEOS in rotations.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Gets the target velocity of the NEOS in rotations per minute.
     * @return The target velocity of the NEOS in rotations per minute.
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void addFollower(NEOS follower) {
        addFollower(follower, false);
    }

    public void addFollower(NEOS follower, boolean invert) {
        followers.add(follower);
        follower.follow(this);
    }

    /**
     * Gets the proportional gain constant for PIDFF controller.
     * @return The proportional gain constant for PIDFF controller.
     */
    public double getP() {
        return pidController.getP();
    }

    /**
     * Gets the integral gain constant for PIDFF controller.
     * @return The integral gain constant for PIDFF controller.
     */
    public double getI() {
        return pidController.getI();
    }

    /**
     * Gets the derivative gain constant for PIDFF controller.
     * @return The derivative gain constant for PIDFF controller.
     */
    public double getD() {
        return pidController.getD();
    }


    /**
     * Gets the I-Zone constant for PIDFF controller.
     * @return The I-Zone constant for PIDFF control.
     */
    public double getIZ() {
        return pidController.getIZone();
    }

    /**
     * Gets the feedforward gain constant for PIDFF controller.
     * @return The feedforward gain constant for PIDFF controller.
     */
    public double getFF() {
        return pidController.getFF();
    }

    // Documentation: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    public REVLibError changeStatusFrame(StatusFrame frame, int period) {
        REVLibError error = setPeriodicFramePeriod(frame.getFrame(), period);

        return error;
    }

    public REVLibError resetStatusFrame(StatusFrame frame) {
        return changeStatusFrame(frame, frame.getDefaultPeriod());
    }

    // Documentation: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    public enum StatusFrame {
        APPLIED_FAULTS_FOLLOWER(PeriodicFrame.kStatus0, 10),
        VELO_TEMP_VOLTAGE_CURRENT(PeriodicFrame.kStatus1, 20),
        POSITION(PeriodicFrame.kStatus2, 20),
        ANALOG_VOLTAGE_VELO_POS(PeriodicFrame.kStatus3, 50),
        ALTERNATE_VELO_POS(PeriodicFrame.kStatus4, 20),
        ABSOLUTE_ENCODER_POS(PeriodicFrame.kStatus5, 200),
        ABSOLUTE_ENCODER_VELO(PeriodicFrame.kStatus6, 200);

        private final PeriodicFrame frame;
        private final int defaultPeriod; // ms
        StatusFrame(PeriodicFrame frame, int defaultPeriod) {
            this.frame = frame;
            this.defaultPeriod = defaultPeriod;
        }

        public PeriodicFrame getFrame() {
            return frame;
        }

        public int getDefaultPeriod() {
            return defaultPeriod;
        }
    }

    public enum ControlLoopType {
        POSITION,
        VELOCITY,
        PERCENT;
    }
}
