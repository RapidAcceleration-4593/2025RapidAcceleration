package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor = ElevatorConstants.elevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    public final Encoder elevatorEncoder = ElevatorConstants.elevatorEncoder;

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                                ElevatorConstants.ELEVATOR_PID.kD,
                                                                                new TrapezoidProfile.Constraints(
                                                                                    ElevatorConstants.MAX_VELOCITY, 
                                                                                    ElevatorConstants.MAX_ACCELERATION));

    private final double[] SETPOINTS = {0, 3000, 12000};

    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the ElevatorSubsystem class.
     * Configures motor settings and establishes leader-follower configuration.
     */
    public ElevatorSubsystem() {
        config.idleMode(IdleMode.kBrake);
        
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorPID.setTolerance(ElevatorConstants.PID_TOLERANCE);
        elevatorPID.reset(0);
    }


    /** ----- Elevator State Management ----- */

    /**
     * Retrieves the corresponding encoder setpoint for the given elevator state.
     * @param state Desired elevator state.
     * @return The target encoder value.
     */
    private double getElevatorState(ElevatorStates state) {
        return switch(state) {
            case BOTTOM -> SETPOINTS[0];
            case PICKUP -> SETPOINTS[1];
            case L4 -> SETPOINTS[2];
            default -> -1;
        };
    }

    /**
     * Sets the target elevator position based on the given state.
     * @param state Desired elevator state.
     */
    public void setElevatorState(ElevatorStates state) {
        double target = getElevatorState(state);
        elevatorPID.setGoal(target);
        SmartDashboard.putNumber("E-Setpoint", target);
    }

    /** ----- Elevator State System ----- */

    /**
     * Controls elevator movement based on limit switch inputs and encoder feedback.
     * <ul>
     *  <li>Stops the motors if both the top and bottom limit switches are pressed (i.e. system malfunction).</li>
     *  <li>Handles specific behavior when either the top or bottom limit switch is pressed.</li>
     *  <li>Uses Profiled PID Control to adjust the motor output when no limit switches are triggered.</li>
     * </ul>
     */
    public void controlElevatorState(boolean usePID) {
        updateValues();

        if (usePID) {
            if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
                stopMotor();
            } else if (isTopLimitSwitchPressed()) {
                handleTopLimitSwitchPressed();
            } else if (isBottomLimitSwitchPressed()) {
                handleBottomLimitSwitchPressed();
            } else {
                controlElevator();
            }
        }
    }

    /** Controls the Elevator System using Profiled PID Control. */
    private void controlElevator() {
        double output = elevatorPID.calculate(getEncoderValue());
        boolean atSetpoint = elevatorPID.atGoal();

        if (atSetpoint) {
            stopMotor();
        } else {
            setMotorSpeed(output);
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
        double currentPosition = getEncoderValue();
        double setpoint = getSetpoint();

        if (setpoint >= currentPosition) {
            stopMotor();
            elevatorPID.setGoal(currentPosition);
            elevatorPID.reset(currentPosition);
        } else {
            controlElevator();
        }
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder, stops the motors, and sets the current position as the new setpoint.</li>
     *  <li>Allows upward movement without interference.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        double setpoint = getSetpoint();

        resetEncoder();

        if (setpoint <= SETPOINTS[0]) {
            stopMotor();
            elevatorPID.setGoal(0);
            elevatorPID.reset(0);
        } else {
            controlElevator();
        }
    }


    /** ----- Encoder and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ElevatorSubsystem#topLimitSwitch} is pressed.
     */
    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ElevatorSubsystem#bottomLimitSwitch} is pressed.
     */
    public boolean isBottomLimitSwitchPressed() {
        return bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speed for both motors; {@link ElevatorSubsystem#leftElevatorFollower} is mirrored.
     * @param speed The speed value for the elevator motors.
     */
    public void setMotorSpeed(double speed) {
        elevatorMotor.set(-speed);
    }

    /** Stops movement for the elevator motors. */
    public void stopMotor() {
        elevatorMotor.stopMotor();
    }

    /**
     * Retrieves the current encoder value of the elevator.
     * @return The current encoder value of the elevator.
     */
    private double getEncoderValue() {
        return -elevatorEncoder.get();
    }

    /** Resets the elevator encoder. */
    private void resetEncoder() {
        elevatorEncoder.reset();
    }

    /**
     * Gets the setpoint for the Elevator PID Controller.
     * @return The current numerical setpoint value.
     */
    private double getSetpoint() {
        return elevatorPID.getGoal().position;
    }

    /**
     * Updates values to SmartDashboard/ShuffleBoard.
     */
    private void updateValues() {
        SmartDashboard.putBoolean("E-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("E-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("E-Encoder", getEncoderValue());
    }
}
