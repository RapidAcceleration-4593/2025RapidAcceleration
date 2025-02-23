package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor = ArmConstants.armMotor;
    private final Encoder armEncoder = ArmConstants.armEncoder;

    private final DigitalInput topLimitSwitch = ArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;
    
    private final PIDController armPID = new PIDController(ArmConstants.ARM_PID.kP,
                                                            ArmConstants.ARM_PID.kI,
                                                            ArmConstants.ARM_PID.kD);

    private final ArmFeedforward armFeedforward = new ArmFeedforward(0.15, 0.15, 0.5);

    private final double[] SETPOINTS = {-20, 600, 900};

    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the SwingArmSubsystem class.
     * Initializes the motor and encoder configuration.
     */
    public ArmSubsystem() {
        config.idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPID.setTolerance(ArmConstants.PID_TOLERANCE);
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
            default -> -1;
        };
    }

    /**
     * Sets the setpoint of the arm to the target state.
     * @param state The desired arm position.
     */
    public void setArmState(ArmStates state) {
        double target = getArmState(state);
        armPID.setSetpoint(target);
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
        } else {
            controlArm();
        }
    }

    /** Controls the Arm System using a PID Controller. */
    private void controlArm() {
        double pidOutput = armPID.calculate(getEncoderValue());
        double ffOutput = armFeedforward.calculate(1.58, 0);

        if (atSetpoint()) {
            stopMotor();
        } else {
            setMotorSpeed(pidOutput + ffOutput);
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
        double currentPosition = getEncoderValue();

        if (getSetpoint() >= currentPosition) {
            stopMotor();
            armPID.setSetpoint(currentPosition);
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
        double setpoint = getSetpoint();
        
        resetEncoder();

        if (setpoint <= SETPOINTS[0]) {
            armPID.setSetpoint(0);
            stopMotor();
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
        return armPID.getSetpoint();
    }

    /**
     * Whether the arm is at its setpoint.
     * @return If the arm is at the setpoint, accounting for tolerance.
     */
    public boolean atSetpoint() {
        return armPID.atSetpoint();
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

    /** Rotates the arm mechanism down to score on a branch. */
    public void placeCoralCommand() {
        armPID.setSetpoint(getSetpoint() - ArmConstants.PLACE_ROTATION_AMOUNT);
    }

    /** Rotates the arm mechanism up to remove algae. */
    public void removeAlgaeCommand() {
        armPID.setSetpoint(getSetpoint() + ArmConstants.PLACE_ROTATION_AMOUNT);
    }

    public Command GoToStateCommand(ArmStates state) {
        return new FunctionalCommand(
            () -> setArmState(state),
            () -> controlArmState(),
            interrupted -> stopMotor(),
            () -> atSetpoint(),
            this
        );
    }

    public Command manualArmCommand(DoubleSupplier controlInput) {
        return new FunctionalCommand(
            null,
            () -> {
                if (isBottomLimitSwitchPressed() || isTopLimitSwitchPressed())
                    stopMotor();
                else
                    setMotorSpeed(controlInput.getAsDouble());
            },
            (interrupted) -> {
                armPID.setSetpoint(getEncoderValue());
                stopMotor();
            },
            null,
            this
        );
    }
}
