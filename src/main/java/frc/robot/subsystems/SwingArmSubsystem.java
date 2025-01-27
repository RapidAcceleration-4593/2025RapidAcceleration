package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwingArmConstants;

public class SwingArmSubsystem extends SubsystemBase {

    private final SparkMax swingArmMotor = SwingArmConstants.swingArmMotor;
    private final Encoder swingArmEncoder = SwingArmConstants.swingArmEncoder;

    private final DigitalInput topLimitSwitch = SwingArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = SwingArmConstants.bottomLimitSwitch;

    private final PIDController upwardPID = new PIDController(
        SwingArmConstants.UPWARD_PID.kP,
        SwingArmConstants.UPWARD_PID.kI,
        SwingArmConstants.UPWARD_PID.kD
    );

    private final PIDController downwardPID = new PIDController(
        SwingArmConstants.DOWNWARD_PID.kP,
        SwingArmConstants.DOWNWARD_PID.kI,
        SwingArmConstants.DOWNWARD_PID.kD
    );
    
    private final PIDController holdPID = new PIDController(
        SwingArmConstants.HOLD_PID.kP,
        SwingArmConstants.HOLD_PID.kI,
        SwingArmConstants.HOLD_PID.kD
    );

    /** Backing variable for {@link #getCurrentPID()}. It should not be used directly. */
    private PIDController currentPID = null;
    
    /** Backing variable for {@link #getArmSetpoint()} and {@link #setArmSetpoint()}. It should not be used directly. */
    private double armSetpoint = 0;

    private final double[] setpoints = {0, 0, 0, 0}; // TODO: Determine setpoint values.
    private SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the SwingArmSubsystem class.
     * Initializes the motor and encoder configuration.
     */
    public SwingArmSubsystem() {
        config.idleMode(IdleMode.kBrake);
        swingArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- PID Controller Management ----- */

    /**
     * Returns the PID controller that should be used based on whether the arm is moving up, down, or is holding. <p>
     * NEVER modifiy the returned controller's characteristics. Instead directly modify {@link #downwardPID}, {@link #upwardPID}, and {@link #holdPID}.
     * To get and set the returned controller's setpoint, use {@link #getArmSetpoint()} and {@link #setArmSetpoint()}.
    */
    private PIDController getCurrentPID() {
        double currentPosition = readEncoderNormalized();

        if (getArmSetpoint() < currentPosition - SwingArmConstants.DOWNWARD_PID_THRESHOLD) {
            currentPID = downwardPID;
        } else if (getArmSetpoint() > currentPosition + SwingArmConstants.UPWARD_PID_THRESHOLD) {
            currentPID = upwardPID;
        } else if (Math.abs(getArmSetpoint() - currentPosition) < SwingArmConstants.HOLD_PID_THRESHOLD) {
            // TODO: Condition may be a bad check. Perhaps it should have to remain a certain amount of time within this threshold for holdPID to kick in.
            currentPID = holdPID;
        }

        currentPID.setSetpoint(armSetpoint);
        return currentPID;
    }
    

    /** ----- Arm State Management ----- */

    /**
     * Retrieves the setpoint for the specified SwingArmState.
     * @param state The desired arm position.
     * @return The {@link SwingArmSubsystem#setpoints} value corresponding to the state.
     */
    private double getArmStateSetpoint(SwingArmConstants.SwingArmState state) {
        return switch (state) {
            case BOTTOM -> setpoints[0];
            case L1 -> setpoints[1];
            case L2 -> setpoints[2];
            case L3 -> setpoints[3];
            case L4 -> setpoints[4];
            default -> -1;
        };
    }

    /**
     * Moves the arm to a pre-defined position based on the provided state.
     * @param state The target SwingArmState.
     */
    public void moveToState(SwingArmConstants.SwingArmState state) {
        setArmSetpoint(getArmStateSetpoint(state));
    }

    /**
     * Controls the state of the swing arm based on the limit switch inputs and encoder feedback.
     * <p>If no limit switches are pressed, then use regular PID control.</p>
     */
    public void controlArmState() {
        if (!handleLimitSwitchSafety(true)) {
            swingArmMotor.set(getCurrentPID().calculate(readEncoderNormalized()));
        }   
    }


    /** ----- Encoder and Limit Switch Management ----- */

    /**
     * Normalize the encoder reading to a positive value.
     * @return The positive encoder reading.
     */
    private double readEncoderNormalized() {
        return -swingArmEncoder.get();
    }

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link SwingArmSubsystem#topLimitSwitch} is pressed.
     */
    private boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link SwingArmSubsystem#bottomLimitSwitch} is pressed.
     */
    private boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Checks if the top or bottom limit switch is triggered, and stops the motor accordingly. It can do this in two ways:
     * <ol>
     *  <li>Control the PID setpoint and check for bad PID output that would cause the motor to drive into the limit switch.
     *  <li>Directly read the motor speed and check for a motor speed that would cause the motor to drive into the limit switch.
     * </ol>
     * This is controlled by the paramater {@link usePID}. {@link usePID} should be false if the motor speed is being directly
     * controlled and true if it is being controlled by PID.
     * @param usePID If true, this method will use method 1. Otherwise, method 2.
     * @return Whether any limit switch is pressed.
     */
    private boolean handleLimitSwitchSafety(boolean usePID) {
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            swingArmMotor.stopMotor();
            return true;
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed(usePID);
            return true;
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed(usePID);
            return true;
        }

        return false;
    }

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Ensures the arm's PID setpoint doesn't go above the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed(boolean usePID) {
        double currentPosition = readEncoderNormalized();
        if (getArmSetpoint() > currentPosition) {
            setArmSetpoint(currentPosition);
        }

        // TODO: Confirm that on the new robot positive motor speed moves the arm upwards.
        if (!usePID && getMotorSpeed() > SwingArmConstants.LS_PID_THRESHOLD) {
            swingArmMotor.stopMotor();
            return;
        }

        double pidOutput = getCurrentPID().calculate(currentPosition);
        swingArmMotor.set(pidOutput > SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
    }
    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder.</li>
     *  <li>Ensures the arm's PID setpoint doesn't go below the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed(boolean usePID) {
        resetArmEncoder();

        double currentPosition = readEncoderNormalized();
        if (getArmSetpoint() < currentPosition) {
            setArmSetpoint(currentPosition);
        }
        
        // TODO: Confirm that on the new robot negative motor speed moves the arm downwards.
        if (!usePID && getMotorSpeed() < -SwingArmConstants.LS_PID_THRESHOLD) {
            swingArmMotor.stopMotor();
            return;
        }

        double pidOutput = currentPID.calculate(currentPosition);
        swingArmMotor.set(pidOutput < -SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
    }

    /**
     * Sets the setpoint of the arm PID controller.
     * @param The setpoint of the arm PID controller.
     */
    private void setArmSetpoint(double setpoint) {
        armSetpoint = setpoint;
    }

    /**
     * Retrieves the current arm PID setpoint.
     * @return The current setpoint of the arm PID controller.
     */
    private double getArmSetpoint() {
        return armSetpoint;
    }

    /** 
     * Retrieves the current speed of the motor. 
     * @returns The current set speed of the {@link #swingArmMotor}.
    */
    private double getMotorSpeed() {
        return swingArmMotor.get();
    }

    /** Resets the arm encoder. */
    private void resetArmEncoder() {
        swingArmEncoder.reset();
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Returns a command to move the arm up.
     * @return The command to move the arm up.
     */
    public Command moveArmUpCommand() {
        return runOnce(() -> moveArmUp());
    }

    /**
     * Returns a command to move the arm down.
     * @return The command to move the arm down.
     */
    public Command moveArmDownCommand() {
        return runOnce(() -> moveArmDown());
    }

    /**
     * Returns a command to stop the arm.
     * @return The command to stop the arm.
     */
    public Command stopArmCommand() {
        return runOnce(() -> stopArm());
    }

    /**
     * Returns a command to move the arm to the setpoint using PID and limit switches.
     * @return The command to move the arm.
     */
    public Command controlArmStatePIDCommand () {
        return run(() -> controlArmState());
    }

    /**
     * Returns a command to set the arm's setpoint to a target position.
     * @param state The state to move to.
     * @return A command that you should really schedule.
     */
    public Command moveToPositionCommand(SwingArmConstants.SwingArmState state) {
        return runOnce(() -> moveToState(state));
    }

    /** Controls the movement of the arm directly by setting the motor speed. */
    private void controlArmMovement(double speed) {
        if (handleLimitSwitchSafety(false))
            return;
        
        setArmSetpoint(readEncoderNormalized());
        swingArmMotor.set(speed);
    }

    /** Moves the swing arm up by directly running the motor. */
    private void moveArmUp() {
        controlArmMovement(SwingArmConstants.MANUAL_CONTROL_SPEED);
    }

    /** Moves the swing arm down by directly running the motor. */
    private void moveArmDown() {
        controlArmMovement(-SwingArmConstants.MANUAL_CONTROL_SPEED);
    }

    /** Stops the movement of the swing arm. */
    private void stopArm() {
        setArmSetpoint(readEncoderNormalized());
        swingArmMotor.stopMotor();
    }
}
