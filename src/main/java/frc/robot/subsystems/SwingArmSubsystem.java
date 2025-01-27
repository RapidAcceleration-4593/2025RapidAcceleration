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
     * Controls the state of the elevator system based on the limit switch inputs and encoder feedback.
     * <ul>
     *  <li>Stops the motor if both the top and bottom limit switches are pressed (i.e. system malfunction).</li>
     *  <li>Handles specific behavior when either the top or bottom limit switch is pressed.</li>
     *  <li>Uses PID control to adjust the motor output when no limit switches are triggered.</li>
     * </ul>
     */
    public void controlElevatorState() {
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            setMotorSpeeds(0);
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else {
            // Regular PID Control.
            setMotorSpeeds(getCurrentPID().calculate(readEncoderNormalized()));
        }
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Ensures the elevator's PID setpoint doesn't go above the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed() {
        double currentPosition = readEncoderNormalized();
        if (getArmSetpoint() > currentPosition) {
            setArmSetpoint(currentPosition);
        }

        double pidOutput = getCurrentPID().calculate(currentPosition);
        setMotorSpeeds(pidOutput > SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
    }
    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder.</li>
     *  <li>Ensures the elevator's PID setpoint doesn't go below the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        resetHeightEncoder();

        double currentPosition = readEncoderNormalized();
        if (getArmSetpoint() < currentPosition) {
            setArmSetpoint(currentPosition);
        }

        double pidOutput = currentPID.calculate(currentPosition);
        setMotorSpeeds(pidOutput < SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
    }

    /**
     * Sets the motor speed for both motors; {@link ElevatorSubsystem#leftElevatorMotor} is mirrored.
     * @param speed The speed value for the elevator motors.
     */
    private void setMotorSpeeds(double speed) {
        // TODO: Determine which motor to invert.
        armMotor.set(speed);
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

    /** Resets the height encoder. */
    private void resetHeightEncoder() {
        armEncoder.reset();
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Returns a command to move the elevator up.
     * @return The command to move the elevator up.
     */
    public Command moveElevatorUpCommand() {
        return runOnce(() -> moveElevatorUp());
    }

    /**
     * Returns a command to move the elevator down.
     * @return The command to move the elevator down.
     */
    public Command moveElevatorDownCommand() {
        return runOnce(() -> moveElevatorDown());
    }

    /** Moves the elevator up by increasing the setpoint. */
    private void moveElevatorUp() {
        elevatorPID.setSetpoint(getElevatorSetpoint() + ElevatorConstants.MANUAL_CONTROL_SPEED);
    }

    /** Moves the elevator down by decreasing the setpoint. */
    private void moveElevatorDown() {
        elevatorPID.setSetpoint(getElevatorSetpoint() - ElevatorConstants.MANUAL_CONTROL_SPEED);
    }
}
