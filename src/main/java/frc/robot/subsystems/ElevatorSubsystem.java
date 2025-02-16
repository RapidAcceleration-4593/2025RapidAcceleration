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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax rightElevatorLeader = ElevatorConstants.rightElevatorMotor;
    private final SparkMax leftElevatorFollower = ElevatorConstants.leftElevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final Encoder heightEncoder = ElevatorConstants.heightEncoder;

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                                ElevatorConstants.ELEVATOR_PID.kD,
                                                                                new TrapezoidProfile.Constraints(
                                                                                    ElevatorConstants.MAX_VELOCITY, 
                                                                                    ElevatorConstants.MAX_ACCELERATION));

    private final double[] setpoints = {0, 2000, 6000}; // TODO: Determine Setpoint Values.

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    private Timer exampleTimer = new Timer();

    /**
     * Constructor for the ElevatorSubsystem class.
     * Configures motor settings, establishing leader-follower configuration.
     */
    public ElevatorSubsystem() {
        leaderConfig.idleMode(IdleMode.kBrake);
        followerConfig.idleMode(IdleMode.kBrake).follow(rightElevatorLeader, true);
        
        rightElevatorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftElevatorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorPID.setTolerance(ElevatorConstants.PID_TOLERANCE);
        elevatorPID.reset(0);
    }


    /** ----- Elevator State Management ----- */

    /**
     * Retrieves the corresponding encoder setpoint for the given elevator state.
     * @param state Desired elevator state.
     * @return The target encoder value.
     */
    private double getElevatorStates(ElevatorStates state) {
        return switch (state) {
            case BOTTOM -> setpoints[0];
            case PICKUP -> setpoints[1];
            case L4 -> setpoints[2];
            default -> -1;
        };
    }

    /**
     * Updates the elevator's target position.
     * @param state Desired elevator state.
     */
    public void setElevatorSetpoint(ElevatorStates state) {
        double target = getElevatorStates(state);
        elevatorPID.setGoal(target);
        SmartDashboard.putNumber("E-Setpoint", target);
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
            controlElevator();
        }
    }

    /** Controls the elevator system using PID control. */
    private void controlElevator() {
        double output = elevatorPID.calculate(getEncoderValue());
        boolean atSetpoint = elevatorPID.atGoal();

        if (atSetpoint) {
            stopElevatorMotors();
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
        stopElevatorMotors();
        elevatorPID.reset(getEncoderValue());
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder and stops the motors.</li>
     *  <li>Allows upward movement without interference.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        stopElevatorMotors();
        resetHeightEncoder();
        elevatorPID.reset(0);
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
        rightElevatorLeader.set(speed);
    }

    /**
     * Stops the elevator motors.
     * <p>Both motors are stopped to prevent any movement.</p>
     */
    public void stopElevatorMotors() {
        rightElevatorLeader.stopMotor();
    }

    /**
     * Retrieves the current encoder value of the elevator.
     * @return The current encoder value of the elevator.
     */
    private double getEncoderValue() {
        SmartDashboard.putNumber("E-Encoder", -heightEncoder.get());
        return -heightEncoder.get();
    }

    /** Resets the height encoder. */
    private void resetHeightEncoder() {
        SmartDashboard.putNumber("E-Encoder", 0);
        heightEncoder.reset();
    }


    /** ----- Experimental Section ----- */
    public void testVelocity() {
        exampleTimer.start();
        setMotorSpeeds(1.0);

        SmartDashboard.putNumber("VelocityTime", exampleTimer.get());
        SmartDashboard.putNumber("VelocitySpeed", heightEncoder.getRate());

        // Velocity = Graph's maximum asymptote.
        // Acceleration = Where the derivative is zero and is a maximum/concave down.

        if (exampleTimer.get() >= 1.0) {
            exampleTimer.stop();
            stopElevatorMotors();
            exampleTimer.reset();
        }
    }
}
