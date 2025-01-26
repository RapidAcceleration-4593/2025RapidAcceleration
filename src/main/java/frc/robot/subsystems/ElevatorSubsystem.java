package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax leftElevatorMotor = ElevatorConstants.leftElevatorMotor;
    private final SparkMax rightElevatorMotor = ElevatorConstants.rightElevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final Encoder heightEncoder = ElevatorConstants.heightEncoder;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    private SparkMaxConfig config = new SparkMaxConfig();

    // TODO: Determine setpoint values.
    private final double[] setpoints = {0, 0, 0, 0};

    /**
     * Constructor for the ElevatorSubsystem class.
     * Initializes the motor and encoder configuration.
     */
    public ElevatorSubsystem() {
        config.idleMode(IdleMode.kBrake);

        leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Elevator State Management ----- */

    /**
     * Retrieves the setpoint for the specified elevator level.
     * @param level The desired elevator level.
     * @return The {@link ElevatorSubsystem#setpoints} value corresponding to the level.
     */
    private double getElevatorStateSetpoint(ElevatorConstants.ElevatorLevel level) {
        return switch (level) {
            case BOTTOM -> setpoints[0];
            case PICKUP -> setpoints[1];
            case L3 -> setpoints[2];
            case L4 -> setpoints[3];
            default -> -1;
        };
    }

    /**
     * Raises the elevator to a specified level.
     * @param level The target elevator state/level.
     */
    public void raiseElevatorToLevel(ElevatorConstants.ElevatorLevel level) {
        elevatorPID.setSetpoint(getElevatorStateSetpoint(level));
    }


    /** ----- Encoder and Limit Switch Management ----- */

    /**
     * Normalize the encoder reading to a positive value.
     * @return The positive encoder reading.
     */
    private double readEncoderNormalized() {
        return -heightEncoder.get();
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
            setMotorSpeeds(elevatorPID.calculate(readEncoderNormalized()));
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
        if (getElevatorSetpoint() > currentPosition) {
            elevatorPID.setSetpoint(currentPosition);
        }

        double pidOutput = elevatorPID.calculate(currentPosition);
        setMotorSpeeds(pidOutput > ElevatorConstants.PID_THRESHOLD ? 0 : pidOutput);
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
        if (getElevatorSetpoint() < currentPosition) {
            elevatorPID.setSetpoint(currentPosition);
        }

        double pidOutput = elevatorPID.calculate(currentPosition);
        setMotorSpeeds(pidOutput < ElevatorConstants.PID_THRESHOLD ? 0 : pidOutput);
    }

    /**
     * Sets the motor speed for both motors; {@link ElevatorSubsystem#leftElevatorMotor} is mirrored.
     * @param speed The speed value for the elevator motors.
     */
    private void setMotorSpeeds(double speed) {
        // TODO: Determine which motor to invert.
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(-speed);
    }

    /**
     * Retrieves the current elevator setpoint.
     * @return The current setpoint of the elevator PID controller.
     */
    private double getElevatorSetpoint() {
        return elevatorPID.getSetpoint();
    }

    /** Resets the height encoder. */
    private void resetHeightEncoder() {
        heightEncoder.reset();
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
