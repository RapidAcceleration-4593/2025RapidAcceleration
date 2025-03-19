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
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Constants.RobotStates.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax leaderElevatorMotor = ElevatorConstants.rightElevatorMotor;
    private final SparkMax followerElevatorMotor = ElevatorConstants.leftElevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final Encoder elevatorEncoder = ElevatorConstants.elevatorEncoder;

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorPIDConstants.ELEVATOR_PID.kP,
                                                                                ElevatorPIDConstants.ELEVATOR_PID.kI,
                                                                                ElevatorPIDConstants.ELEVATOR_PID.kD,
                                                                                new TrapezoidProfile.Constraints(
                                                                                    ElevatorPIDConstants.MAX_VELOCITY, 
                                                                                    ElevatorPIDConstants.MAX_ACCELERATION));

    private static final double[] SETPOINTS = {-500, 3250, 12550};

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    /**
     * Returns if the elevator is solely in manual mode.
     * PID is completely disabled. Access only through {@link #isManualControlEnabled()}.
     */
    private boolean manualControlEnabled = false;

    private ElevatorStates simulatedState = ElevatorStates.BOTTOM;

    /**
     * Constructor for the ElevatorSubsystem class.
     * Configures motor settings and establishes leader-follower configuration.
     */
    public ElevatorSubsystem() {
        leaderConfig.idleMode(IdleMode.kBrake);
        followerConfig.idleMode(IdleMode.kBrake).follow(leaderElevatorMotor, true);
        
        leaderElevatorMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerElevatorMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorPID.setTolerance(ElevatorPIDConstants.TOLERANCE);
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
            default -> throw new Error("Passed in an ElevatorState that does not have an associated setpoint!");
        };
    }

    /**
     * Sets the target elevator position based on the given state.
     * @param state Desired elevator state.
     */
    public void setElevatorState(ElevatorStates state) {
        elevatorPID.setGoal(getElevatorState(state));

        if (Robot.isSimulation()) {
            simulatedState = state;
        }
    }

    /**
     * Gets the current elevator state based on the encoder values.
     * @return The current elevator state.
     */
    public ElevatorStates getCurrentState() {
        if (Robot.isSimulation())  {
            return simulatedState;
        }

        if (getEncoderValue() <= SETPOINTS[0] + 500) {
            return ElevatorStates.BOTTOM;
        } else if (getEncoderValue() <= SETPOINTS[1] + 500) {
            return ElevatorStates.PICKUP;
        } else {
            return ElevatorStates.TOP;
        }
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

        if (isManualControlEnabled()) {
            return;
        }

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
     *  <li>If {@link #hardManualEnabled}, stops the motor if it is driving upward and returns.
     *  <li>Stops the motors and sets the current position as the new setpoint.</li>
     *  <li>Allows downward movement without interference.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed() {
        if (getSetpoint() >= getEncoderValue()) {
            stopMotors();
            resetSetpoint(getEncoderValue());
        } else {
            controlElevator();
        }
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>If {@link #hardManualEnabled}, stops the motor if it is driving downward and returns.
     *  <li>Resets the encoder, stops the motors, and sets the current position as the new setpoint.</li>
     *  <li>Allows upward movement without interference.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        resetEncoder();

        if (getSetpoint() <= 0) {
            stopMotors();
            resetSetpoint(0);
        } else {
            controlElevator();
        }
    }


    /** ----- Motor, Encoder, and Limit Switch Abstraction ----- */

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
    public double getEncoderValue() {
        return -elevatorEncoder.get();
    }

    /** Resets the elevator encoder. */
    public void resetEncoder() {
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


    /** ----- Miscellaneous Methods ----- */

    /**
     * Sets the setpoint for the elevator PID controller, without resetting.
     * @param setpoint New setpoint value.
     */
    public void setSetpoint(double setpoint) {
        elevatorPID.setGoal(setpoint);
    }

    /**
     * Resets the setpoint for the elevator PID controller.
     * @param setpoint New setpoint value.
     */
    public void resetSetpoint(double setpoint) {
        elevatorPID.setGoal(setpoint);
        elevatorPID.reset(setpoint);
    }

    /**
     * Sets {@link #manualControlEnabled}. Setting to true completely disables PID control and changes limit switch behavior.
     * @return Is hard manual control enabled.
     */
    public void setManualControl(boolean enabled) {
        manualControlEnabled = enabled;
    }

    /**
     * Gets {@link #manualControlEnabled}. When true it completely disables PID control and changes limit switch behavior.
     * @return Whether hard manual control enabled.
     */
    public boolean isManualControlEnabled() {
        return manualControlEnabled;
    }

    /** Updates values to SmartDashboard/ShuffleBoard. */
    private void updateValues() {
        SmartDashboard.putBoolean("E-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("E-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("E-Encoder", getEncoderValue());
        SmartDashboard.putNumber("E-Setpoint", getSetpoint());
        SmartDashboard.putString("E-State", getCurrentState().toString());
    }
}
