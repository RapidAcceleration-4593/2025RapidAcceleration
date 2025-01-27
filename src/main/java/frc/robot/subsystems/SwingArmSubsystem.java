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

public class SwingArmSubsystem extends SubsystemBase{
    private final Encoder armEncoder = SwingArmConstants.armEncoder;

    private final SparkMax armMotor = SwingArmConstants.armMotor;

    private final DigitalInput topLimitSwitch = SwingArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = SwingArmConstants.bottomLimitSwitch;

    private final PIDController upwardPID = new PIDController(SwingArmConstants.UPWARD_PID.kP,
                                                              SwingArmConstants.UPWARD_PID.kI,
                                                              SwingArmConstants.UPWARD_PID.kD);
    private final PIDController downwardPID = new PIDController(SwingArmConstants.DOWNWARD_PID.kP,
                                                                SwingArmConstants.DOWNWARD_PID.kI,
                                                                SwingArmConstants.DOWNWARD_PID.kD);
    private final PIDController holdPID = new PIDController(SwingArmConstants.HOLD_PID.kP,
                                                                SwingArmConstants.HOLD_PID.kI,
                                                                SwingArmConstants.HOLD_PID.kD);

    /**This is the backing variable for {@link #getCurrentPID}(). It should not be used directly.*/
    private PIDController currentPID = null;
    
    /**This is the backing variable for {@link #getArmSetpoint}() and {@link #setArmSetpoint}(). It should not be used directly.*/
    private double armSetpoint = 0;

    private SparkMaxConfig config = new SparkMaxConfig();

    // TODO: Determine setpoint values.
    private final double[] setpoints = {0, 0, 0, 0};

    /**
     * Constructor for the SwingArmSubsystem class.
     * Initializes the motor configuration.
     */
    public SwingArmSubsystem() {
        config.idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** ----- PID Controller Management ----- */

    /**
     * Returns the PID controller that should be used based on whether the arm is moving up, down, or is holding. <p>
     * NEVER modifiy the returned controller's characteristics. Instead directly modify {@code downwardPID}, {@code upwardPID}, and {@code holdPID}.
     * To get and set the returned controller's setpoint, use {@link #getArmSetpoint}() and {@link #setArmSetpoint}().
    */
    private PIDController getCurrentPID() {
        if (getArmSetpoint() < readEncoderNormalized() - SwingArmConstants.DOWNWARD_PID_THRESHOLD) {
            currentPID = downwardPID;
        }
        else if (getArmSetpoint() > readEncoderNormalized() + SwingArmConstants.UPWARD_PID_THRESHOLD) {
            currentPID = upwardPID;
        }
        // TODO: This condition may be a bad check. Perhaps it should have to remain a certain amount of time
        //       within this threshold for holdPID to kick in.
        else if (Math.abs(getArmSetpoint() - readEncoderNormalized()) < SwingArmConstants.HOLD_PID_THRESHOLD) {
            currentPID = holdPID;
        }
        currentPID.setSetpoint(armSetpoint);
        return currentPID;
    }

    /** ----- Arm State Management ----- */

    /**
     * Retrieves the setpoint for the specified arm position.
     * @param level The desired arm position.
     * @return The {@link ElevatorSubsystem#setpoints} value corresponding to the level.
     */
    private double getArmStateSetpoint(SwingArmConstants.ArmPosition level) {
        return switch (level) {
            case BOTTOM -> setpoints[0];
            case L3 -> setpoints[2];
            case L4 -> setpoints[3];
            default -> -1;
        };
    }

    /**
     * Raises the elevator to a specified level.
     * @param level The target elevator state/level.
     */
    public void goToPosition(SwingArmConstants.ArmPosition level) {
        setArmSetpoint(getArmStateSetpoint(level));
    }


    /** ----- Encoder and Limit Switch Management ----- */

    /**
     * Normalize the encoder reading to a positive value.
     * @return The positive encoder reading.
     */
    private double readEncoderNormalized() {
        return -armEncoder.get();
    }

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ElevatorSubsystem#topLimitSwitch} is pressed.
     */
    private boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ElevatorSubsystem#bottomLimitSwitch} is pressed.
     */
    private boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**
     * Controls the state of the arm based on the limit switch inputs and encoder feedback.
     * <ul>
     *  <li>Stops the motor if both the top and bottom limit switches are pressed (i.e. system malfunction).</li>
     *  <li>Handles specific behavior when either the top or bottom limit switch is pressed.</li>
     *  <li>Uses PID control to adjust the motor output when no limit switches are triggered.</li>
     * </ul>
     */
    public void controlArmState() {
        
        // If no limit switches pressed
        if (!runLimitSwitchSafety(true)) {
            // Use regular PID Control.
            armMotor.set(getCurrentPID().calculate(readEncoderNormalized()));
        }   
    }

    /** Moves the arm up by directly running the motor. */
    private void moveArmUp() {
        if (runLimitSwitchSafety(false))
            return;

        setArmSetpoint(readEncoderNormalized());
        armMotor.set(SwingArmConstants.ARM_MANUAL_CONTROL_SPEED);
    }

    /** Moves the arm down by directly running the motor. */
    private void moveArmDown() {
        if (runLimitSwitchSafety(false))
            return;

        setArmSetpoint(readEncoderNormalized());
        armMotor.set(-SwingArmConstants.ARM_MANUAL_CONTROL_SPEED);
    }

    private void stopArm() {
        setArmSetpoint(readEncoderNormalized());
        armMotor.set(0);
    }

    /** ----- Limit Switch Handling ----- */

    /**
     * Checks if the top or bottom limit switch is triggered, and stops the motor accordingly. It can 
     * do this in two ways:
     * <ol>
     *  <li>Control the PID setpoint and check for bad PID output that would cause the motor to drive into the limit switch.
     *  <li>Directly read the motor speed and check for a motor speed that would cause the motor to drive into the limit switch.
     * </ol>
     * This is controlled by the paramater {@code usePID}. {@code usePID} should be false if the motor speed is being directly
     * controlled and true if it is being controlled by PID.
     * @param usePID If true, this method will use method 1. Otherwise, method 2.
     * @return Whether any limit switch is pressed.
     */
    private boolean runLimitSwitchSafety(boolean usePID) {
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            armMotor.set(0);
            return true;
        }
        else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed(usePID);
            return true;
        }
        else if (isBottomLimitSwitchPressed()) {
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

        // TODO: Confirm that on the new robot positive motor speed moves the arm upwards
        if (!usePID && armMotor.get() > SwingArmConstants.LS_PID_THRESHOLD) {
            armMotor.set(0);
            return;
        }

        double pidOutput = getCurrentPID().calculate(currentPosition);
        armMotor.set(pidOutput > SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
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
        
        // TODO: Confirm that on the new robot negative motor speed moves the arm downwards
        if (!usePID && armMotor.get() < -SwingArmConstants.LS_PID_THRESHOLD) {
            armMotor.set(0);
            return;
        }

        double pidOutput = currentPID.calculate(currentPosition);
        armMotor.set(pidOutput < -SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
    }

    /**
     * Retrieves the current arm PID setpoint.
     * @return The current setpoint of the arm PID controller.
     */
    private double getArmSetpoint() {
        return armSetpoint;
    }

    /**
     * Sets the setpoint of the arm PID controller.
     * @param 
     */
    private void setArmSetpoint(double setpoint) {
        armSetpoint = setpoint;
    }

    /** Resets the arm encoder. */
    private void resetArmEncoder() {
        armEncoder.reset();
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
     * @return I really don't think there is anything more to explain.
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
     * @param position The position to go to.
     * @return A command that you should really schedule.
     */
    public Command moveToPositionCommand(SwingArmConstants.ArmPosition position) {
        return runOnce(() -> goToPosition(position));
    }
}
