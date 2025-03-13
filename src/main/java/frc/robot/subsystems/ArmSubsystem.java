package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.RobotStates.Arm.ArmDirections;
import frc.robot.Constants.RobotStates.Arm.ArmStates;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor = ArmConstants.armMotor;
    private final Encoder armEncoder = ArmConstants.armEncoder;

    private final DigitalInput topLimitSwitch = ArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;
    
    private final ProfiledPIDController armPID = new ProfiledPIDController(ArmPIDConstants.ARM_PID.kP,
                                                                           ArmPIDConstants.ARM_PID.kI,
                                                                           ArmPIDConstants.ARM_PID.kD,
                                                                           new TrapezoidProfile.Constraints(
                                                                                ArmPIDConstants.MAX_VELOCITY,
                                                                                ArmPIDConstants.MAX_ACCELERATION));

    private final double[] SETPOINTS = {-20, 600, 875}; // TODO: Adjust Setpoint Values. Previously: {-20, 600, 900}.

    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Returns if the elevator is solely in manual mode.
     * PID is completely disabled. Access only through {@link #isManualControlEnabled()}.
     */
    private boolean manualControlEnabled = false;

    private ArmStates targetArmState = ArmStates.BOTTOM;

    /**
     * Constructor for the SwingArmSubsystem class.
     * Initializes the motor and encoder configuration.
     */
    public ArmSubsystem() {
        config.idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPID.setTolerance(ArmPIDConstants.TOLERANCE);
        armPID.reset(0);
    }


    /** ----- Arm State Management ----- */

    /**
     * Retrieves the setpoint for the specified SwingArmStates.
     * @param state The desired arm position.
     * @return The {@link ArmSubsystem#SETPOINTS} value corresponding to the state.
     */
    private double getArmState(ArmStates state) {
        return switch (state) {
            case BOTTOM -> SETPOINTS[0];
            case L2 -> SETPOINTS[1];
            case TOP -> SETPOINTS[2];
            default -> throw new Error("Passed in an ArmState that does not have an associated setpoint!");
        };
    }

    /**
     * Sets the setpoint of the arm to the target state.
     * @param state The desired arm position.
     */
    public void setTargetArmState(ArmStates state) {
        armPID.setGoal(getArmState(state));
        targetArmState = state;
    }

    /**
     * Gets the arm state based on the previously set goal.
     * @return The current arm state.
     */
    public ArmStates getTargetArmState() {
        return targetArmState;
    }


    /** ----- Arm State system ----- */

    /**
     * Controls arm movement based on limit switch inputs and encoder feedback.
     * <ul>
     *  <li>Stops the motor if both the top and bottom limit switches are pressed (i.e. system malfunction).</li>
     *  <li>Handles specific behavior when either the top or bottom limit switch is pressed.</li>
     *  <li>Uses PID Control to adjust the motor output when no limit switches are triggered.</li>
     * </ul>
     */
    public void controlArmState() {
        updateValues();

        if (isManualControlEnabled()) return;

        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            stopMotor();
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else {
            controlArm();
        }
    }

    /** Controls the Arm System using a PID Controller. */
    private void controlArm() {
        double output = armPID.calculate(getEncoderValue());

        if (atSetpoint()) {
            stopMotor();
        } else {
            setMotorSpeed(output);
        }
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Stops the motor and sets the current position as the new setpoint.</li>
     *  <li>Allows downward movement without interference.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed() {
        if (getSetpoint() >= getEncoderValue()) {
            stopMotor();
            resetSetpoint(getEncoderValue());
        } else {
            controlArm();
        }
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder, stops the motor, and sets the current position as the new setpoint. </li>
     *  <li>Allows upward movement without interference.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        resetEncoder();
                    
        if (getSetpoint() <= 0) {
            stopMotor();
            resetSetpoint(0);
        } else {
            controlArm();
        }
    }


    /** ----- Motor, Encoder, and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ArmSubsystem#topLimitSwitch} is pressed.
     */
    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ArmSubsystem#bottomLimitSwitch} is pressed.
     */
    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speed for the arm motor.
     * @param speed The desired speed of the motor.
     */
    public void setMotorSpeed(double speed) {
        armMotor.set(speed);
    }

    /** Stops movement for the arm motor. */
    public void stopMotor() {
        armMotor.stopMotor();
    }

    /**
     * Retrieves the current encoder value of the arm.
     * @return The current encoder value of the arm.
     */
    public double getEncoderValue() {
        return armEncoder.get();
    }

    /** Resets the arm encoder. */
    public void resetEncoder() {
        armEncoder.reset();
    }

    /**
     * Gets the setpoint for the Arm PID Controller.
     * @return The current numerical setpoint value.
     */
    public double getSetpoint() {
        return armPID.getGoal().position;
    }

    /**
     * Whether the arm is at its setpoint.
     * @return If the arm is at the setpoint, accounting for tolerance.
     */
    public boolean atSetpoint() {
        return armPID.atGoal();
    }

    /**
     * Returns if the elevator is above the INTAKE position. This can be used to coordinate elevator/arm movements.
     * Returns {@link ArmDirections#UNKNOWN} if it is unsure whether the arm is up or not.
     * @return Is the elevator at or above the INTAKE position.
     */
    public ArmDirections isArmUp() {
        if (Robot.isSimulation()) return (targetArmState == ArmStates.BOTTOM) ? ArmDirections.DOWN : ArmDirections.UP;

        if (getEncoderValue() <= 50)
            return ArmDirections.DOWN;
        else if (getEncoderValue() >= 500)
            return ArmDirections.UP;
        else
            return ArmDirections.UNKNOWN;
    }


    /** ----- Miscellaneous Methods ----- */

    /**
     * Sets the setpoint for the arm PID Controller, without resetting.
     * @param setpoint New setpoint value.
     */
    public void setSetpoint(double setpoint) {
        armPID.setGoal(setpoint);
    }

    /**
     * Resets the setpoint for the arm PID controller.
     * @param setpoint New setpoint value.
     */
    public void resetSetpoint(double setpoint) {
        armPID.setGoal(setpoint);
        armPID.reset(setpoint);
    }

    /**
     * Sets {@link #manualControlEnabled}. Setting to true completely disables PID control and changes limit switch behavior.
     * @return Is hard manual control enabled.
     */
    public void setManualControl(boolean value) {
        manualControlEnabled = value;
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
        SmartDashboard.putBoolean("A-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("A-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("A-Encoder", getEncoderValue());
        SmartDashboard.putNumber("A-Setpoint", getSetpoint());
        SmartDashboard.putString("A-State", targetArmState.toString());

    }
}
