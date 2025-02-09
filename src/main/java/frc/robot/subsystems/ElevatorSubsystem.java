package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax leftElevatorMotor = ElevatorConstants.leftElevatorMotor;
    private final SparkMax rightElevatorMotor = ElevatorConstants.rightElevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final Encoder heightEncoder = ElevatorConstants.heightEncoder;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    private final double[] setpoints = {0, 750, 1000, 10000}; // TODO: Determine setpoint values.

    private final SparkMaxConfig config = new SparkMaxConfig();

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
     * Sets the elevator state to the specified level.
     * @param state The desired elevator level.
     */
    private double getElevatorStates(ElevatorStates state) {
        return switch (state) {
            case BOTTOM -> setpoints[0];
            case PICKUP -> setpoints[1];
            case L3 -> setpoints[2];
            case L4 -> setpoints[3];
            default -> -1;
        };
    }

    /**
     * Sets the elevator setpoint to the specified level.
     * @param state The desired elevator level.
     */
    public void setElevatorSetpoint(ElevatorStates state) {
        elevatorPID.setSetpoint(getElevatorStates(state));
        SmartDashboard.putNumber("E-Setpoint", getElevatorSetpoint());
    }


    /** ----- Elevator State System ----- */

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
            stopElevatorMotors();
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else {
            controlElevator(getEncoderValue(), getElevatorSetpoint(), ElevatorConstants.PID_THRESHOLD);
        }
    }

    /**
     * Controls the elevator system using PID control.
     * @param encoder The current encoder reading.
     * @param setpoint The desired setpoint for the elevator.
     * @param threshold The threshold for the PID control.
     */
    private void controlElevator(double encoder, double setpoint, double threshold) {
        boolean isWithinThreshold = Math.abs(setpoint - encoder) < threshold;
        boolean isBottomState = setpoint == setpoints[0];
        // TODO: Top State Conditonal.

        if (isWithinThreshold && !isBottomState) {
            // Stop the motors and hold the elevator in place.
            stopElevatorMotors();
        } else {
            // Use PID control to adjust the motor output.
            setMotorSpeeds(elevatorPID.calculate(encoder, setpoint));
        }
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Stops the motors and sets the current position as the new setpoint.</li>
     *  <li>Allows downward movement without interference.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed() {
        if (getElevatorSetpoint() < getEncoderValue() - ElevatorConstants.PID_THRESHOLD) {
            // Allow downward movement without interference.
            setMotorSpeeds(elevatorPID.calculate(getEncoderValue(), getElevatorSetpoint()));
        } else {
            // Stop the motors and set current position as the new setpoint.
            stopElevatorMotors();
            elevatorPID.setSetpoint(getEncoderValue());
        }
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder and stops the motors.</li>
     *  <li>Allows upward movement without interference.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        if (getElevatorSetpoint() > 0 + ElevatorConstants.PID_THRESHOLD) {
            // Allow upward movement without interference.
            setMotorSpeeds(elevatorPID.calculate(getEncoderValue(), getElevatorSetpoint()));
        } else {
            // Reset the encoder and stop the motors.
            resetHeightEncoder();
            stopElevatorMotors();
        }
    }


    /** ----- Encoder and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ElevatorSubsystem#topLimitSwitch} is pressed.
     */
    public boolean isTopLimitSwitchPressed() {
        SmartDashboard.putBoolean("E-TopLS", !topLimitSwitch.get());
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ElevatorSubsystem#bottomLimitSwitch} is pressed.
     */
    public boolean isBottomLimitSwitchPressed() {
        SmartDashboard.putBoolean("E-BotLS", bottomLimitSwitch.get());
        return bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speed for both motors; {@link ElevatorSubsystem#leftElevatorMotor} is mirrored.
     * @param speed The speed value for the elevator motors.
     */
    public void setMotorSpeeds(double speed) {
        leftElevatorMotor.set(-speed);
        rightElevatorMotor.set(speed);
    }

    /**
     * Stops the elevator motors.
     * <p>Both motors are stopped to prevent any movement.</p>
     */
    public void stopElevatorMotors() {
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }

    /**
     * Retrieves the current encoder value of the elevator.
     * @return The current encoder value of the elevator.
     */
    public double getEncoderValue() {
        SmartDashboard.putNumber("E-Encoder", -heightEncoder.get());
        return -heightEncoder.get();
    }

    /**
     * Retrieves the current elevator setpoint.
     * @return The current setpoint of the elevator PID controller.
     */
    public double getElevatorSetpoint() {
        return elevatorPID.getSetpoint();
    }

    /** Resets the height encoder. */
    private void resetHeightEncoder() {
        SmartDashboard.putNumber("E-Encoder", 0);
        heightEncoder.reset();
    }
}
