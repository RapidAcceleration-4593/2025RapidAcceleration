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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_MANUAL_CONTROL;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_MANUAL_CONTROL.ElevatorDirections;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax leaderElevatorMotor = ElevatorConstants.rightElevatorMotor;
    private final SparkMax followerElevatorMotor = ElevatorConstants.leftElevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    public final Encoder elevatorEncoder = ElevatorConstants.elevatorEncoder;

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                                ElevatorConstants.ELEVATOR_PID.kD,
                                                                                new TrapezoidProfile.Constraints(
                                                                                    ElevatorConstants.MAX_VELOCITY, 
                                                                                    ElevatorConstants.MAX_ACCELERATION));

    private final double[] SETPOINTS = {-300, 2750, 12550};

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    /**
     * Constructor for the ElevatorSubsystem class.
     * Configures motor settings and establishes leader-follower configuration.
     */
    public ElevatorSubsystem() {
        leaderConfig.idleMode(IdleMode.kBrake);
        followerConfig.idleMode(IdleMode.kBrake).follow(leaderElevatorMotor, true);
        
        leaderElevatorMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerElevatorMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
            case TOP -> SETPOINTS[2];
            default -> -1;
        };
    }

    /**
     * Sets the target elevator position based on the given state.
     * @param state Desired elevator state.
     */
    public void setElevatorState(ElevatorStates state) {
        elevatorPID.setGoal(getElevatorState(state));
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
    public void controlElevatorState() {
        updateValues();

        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            stopMotors();
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else {
            controlElevator();
        }
    }

    /** Controls the Elevator System using Profiled PID Control. */
    private void controlElevator() {
        double output = elevatorPID.calculate(getEncoderValue());

        if (atSetpoint()) {
            stopMotors();
        } else {
            setMotorSpeeds(output);
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

        if (getSetpoint() >= currentPosition) {
            stopMotors();
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
            stopMotors();
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
     * Sets the motor speed for the elevator motor.
     * @param speed The speed value for the elevator motor.
     */
    public void setMotorSpeeds(double speed) {
        leaderElevatorMotor.set(speed);
    }

    /** Stops movement for the elevator motor. */
    public void stopMotors() {
        leaderElevatorMotor.stopMotor();
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
     * Whether the elevator is at its setpoint.
     * @return If the elevator is at the setpoint, accounting for tolerance.
     */
    public boolean atSetpoint() {
        return elevatorPID.atGoal();
    }

    /**
     * Updates values to SmartDashboard/ShuffleBoard.
     */
    private void updateValues() {
        SmartDashboard.putBoolean("E-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("E-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("E-Encoder", getEncoderValue());
        SmartDashboard.putNumber("E-Setpoint", getSetpoint());
    }

    /**
     * Command to set and control the state of the elevator mechanism.
     * @param state The desired state of the elevator.
     * @return A Command Race to set elevator state with a timeout.
     */
    public Command GoToStateCommand(ElevatorStates state) {
        return Commands.race(
            new FunctionalCommand(
                () -> setElevatorState(state),  // Initialization
                () -> controlElevatorState(),   // Execute
                interrupted -> stopMotors(),     // End
                () -> atSetpoint(),             // IsFinished
                this
            ),
            new WaitCommand(2.25) // Timeout after 3 seconds.
        );
    }

    /**
     * Command to control the elevator manually during PID control.
     * @param direction The direction to manually move the elevator.
     * @return A Functional Command to control the elevator manually.
     */
    public Command manualElevatorCommand(ElevatorDirections direction) {
        return switch (direction) {
            case UP -> new FunctionalCommand(
                ()-> {},
                () -> {
                    if (isTopLimitSwitchPressed()) {
                        stopMotors();
                    } else {
                        setMotorSpeeds(ELEVATOR_MANUAL_CONTROL.MOTOR_SPEED);
                    }
                },
                (interrupted) -> {
                    stopMotors();
                    elevatorPID.setGoal(getEncoderValue());
                    elevatorPID.reset(getEncoderValue());
                },
                () -> false,
                this
            );

            case DOWN -> new FunctionalCommand(
                ()-> {},
                () -> {
                    if (isBottomLimitSwitchPressed()) {
                        stopMotors();
                    } else {
                        setMotorSpeeds(-ELEVATOR_MANUAL_CONTROL.MOTOR_SPEED);
                    }
                },
                (interrupted) -> {
                    stopMotors();
                    elevatorPID.setGoal(getEncoderValue());
                    elevatorPID.reset(getEncoderValue());
                },
                () -> false,
                this
            );
        };
    }

    /**
     * Returns if the elevator is above the INTAKE position. This can be used to coordinate elevator/arm movements.
     * @return Is the elevator at or above the INTAKE position.
     */
    public boolean isUp() {
        return (elevatorPID.atGoal() && elevatorPID.getGoal().position == getElevatorState(ElevatorStates.PICKUP)) || // At INTAKE
            (getEncoderValue() >= getElevatorState(ElevatorStates.PICKUP)); // Above INTAKE
    }
}
