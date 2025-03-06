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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private final double[] SETPOINTS = {-20, 600, 900};

    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Returns if the elevator is solely in manual mode. PID is completely disabled and
     * {@link #handleBottomLimitSwitchPressed()} and {@link #handleTopLimitSwitchPressed()}
     * behave differently. Access through {@link #isHardManualControlEnabled()}.
     */
    private boolean hardManualEnabled = false;

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
    public void setArmState(ArmStates state) {
        armPID.setGoal(getArmState(state));
        SmartDashboard.putString("A-State", state.toString());
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

        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            stopMotor();
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else if (!isHardManualControlEnabled()) {
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
        if (isHardManualControlEnabled()) {
            if (armMotor.get() > 0) {
                stopMotor();
            }
            return;
        }

        if (getSetpoint() >= getEncoderValue()) {
            stopMotor();
            armPID.setGoal(getEncoderValue());
            armPID.reset(getEncoderValue());
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

        if (isHardManualControlEnabled()) {
            if (armMotor.get() < 0) {
                stopMotor();
            }
            return;
        }
                    
        if (getSetpoint() <= 0) {
            stopMotor();
            armPID.setGoal(0);
            armPID.reset(0);
        } else {
            controlArm();
        }
    }


    /** ----- Encoder and Limit Switch Abstraction ----- */

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
    private void resetEncoder() {
        armEncoder.reset();
    }

    /**
     * Gets the setpoint for the Arm PID Controller.
     * @return The current numerical setpoint value.
     */
    private double getSetpoint() {
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
     * Updates values to SmartDashboard/ShuffleBoard.
     */
    private void updateValues() {
        SmartDashboard.putBoolean("A-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("A-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("A-Encoder", getEncoderValue());
        SmartDashboard.putNumber("A-Setpoint", getSetpoint());
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Functional Command to rotate the arm down to score coral.
     * @return A lower setpoint for the arm mechanism.
     */
    public Command scoreCoralCommand() {
        return Commands.race(
            new FunctionalCommand(
                () -> armPID.setGoal(getSetpoint() - ArmConstants.PLACE_ROTATION_AMOUNT),
                () -> controlArmState(),
                interrupted -> stopMotor(),
                () -> atSetpoint(),
                this
            ),
            new WaitCommand(0.5)
        );

    }

    /**
     * Command to set and control the state of the arm mechanism.
     * @param state The desired state of the arm.
     * @return A Command Race to set arm state with a timeout.
     */
    public Command goToStateCommand(ArmStates state) {
        return Commands.race(
            new FunctionalCommand(
                () -> setArmState(state),
                () -> controlArmState(),
                interrupted -> stopMotor(),
                () -> atSetpoint(),
                this
            ),
            new WaitCommand(0.8)
        );
    }

    /**
     * Command to control the arm manually during PID control.
     * @param direction The direction to manually move the arm.
     * @return A Functional Command to control the arm manually.
     */
    public Command manualArmCommand(ArmDirections direction) {
        return switch (direction) {
            case UP -> new FunctionalCommand(
                () -> {},
                () -> {
                    if (isTopLimitSwitchPressed()) {
                        stopMotor();
                    } else {
                        setMotorSpeed(ArmConstants.CONTROL_SPEED);
                    }
                },
                interrupted -> {
                    stopMotor();
                    if (!hardManualEnabled) {
                        armPID.setGoal(getEncoderValue());
                        armPID.reset(getEncoderValue());
                    }
                },
                () -> false,
                this
            );

            case DOWN -> new FunctionalCommand(
                () -> {},
                () -> {
                    if (isBottomLimitSwitchPressed()) {
                        stopMotor();
                    } else {
                        setMotorSpeed(-ArmConstants.CONTROL_SPEED);
                    }
                },
                interrupted -> {
                    stopMotor();
                    if (!hardManualEnabled) {
                        armPID.setGoal(getEncoderValue());
                        armPID.reset(getEncoderValue());
                    }
                },
                () -> false,
                this
            );

            default -> Commands.none();
        };
    }


    /**
     * Returns if the elevator is above the INTAKE position. This can be used to coordinate elevator/arm movements.
     * @return Is the elevator at or above the INTAKE position.
     */
    public ArmDirections isArmUp() {
        if (getEncoderValue() <= 30)
            return ArmDirections.DOWN;
        else if (getEncoderValue() >= 300)
            return ArmDirections.UP;
        else
            return ArmDirections.UNKNOWN;
    }

    /**
     * Sets {@link #hardManualEnabled}. Setting to true completely disables PID control and changes limit switch behavior.
     * @return Is hard manual control enabled.
     */
    public void setHardManualControl(boolean enable) {
        hardManualEnabled = enable;
    }

    /**
     * Gets {@link #hardManualEnabled}. When true it completely disables PID control and changes limit switch behavior.
     * @return Is hard manual control enabled.
     */
    public boolean isHardManualControlEnabled() {
        return hardManualEnabled;
    }
}
