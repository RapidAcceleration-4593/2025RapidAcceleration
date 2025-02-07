package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwingArmConstants;

public class SwingArmSubsystem extends SubsystemBase {

    private final SparkMax swingArmMotor = SwingArmConstants.swingArmMotor;
    private final Encoder swingArmEncoder = SwingArmConstants.swingArmEncoder;

    private final DigitalInput topLimitSwitch = SwingArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = SwingArmConstants.bottomLimitSwitch;
    
    private final PIDController armPID = new PIDController(SwingArmConstants.ARM_PID.kP,
                                                           SwingArmConstants.ARM_PID.kI,
                                                           SwingArmConstants.ARM_PID.kD);

    private final SparkMaxConfig config = new SparkMaxConfig();

    private final double[] setpoints = {0, 0, 0, 0, 0}; // TODO: Determine setpoint values.

    /**
     * Constructor for the SwingArmSubsystem class.
     * Initializes the motor and encoder configuration.
     */
    public SwingArmSubsystem() {
        config.idleMode(IdleMode.kBrake);

        swingArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {
        SmartDashboard.putNumber("ArmEncoder", getEncoderValue());
        SmartDashboard.putBoolean("ArmTopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("ArmBottomLS", isBottomLimitSwitchPressed());

        if (isBottomLimitSwitchPressed()) {
            resetArmEncoder();
        }
    }


    /** ----- Arm State Management ----- */

    /**
     * Retrieves the setpoint for the specified SwingArmStates.
     * @param state The desired arm position.
     * @return The {@link SwingArmSubsystem#setpoints} value corresponding to the state.
     */
    private double getArmStates(SwingArmConstants.SwingArmStates state) {
        return switch (state) {
            case BOTTOM -> setpoints[0];
            case L1 -> setpoints[1];
            case L2 -> setpoints[2];
            case L3 -> setpoints[3];
            case L4 -> setpoints[4];
            default -> -1;
        };
    }

    /**
     * Sets the setpoint of the arm to the target state.
     * @param state The desired arm position.
     */
    public void setArmSetpoint(SwingArmConstants.SwingArmStates state) {
        armPID.setSetpoint(getArmStates(state));
    }


    /** ----- Arm State system ----- */

    /**
     * Controls the state of the swing arm based on the limit switch inputs and encoder feedback.
     * <p>If no limit switches are pressed, then use regular PID control.</p>
     */
    public void controlArmState() {
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            stopArmMotor();
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else {
            controlArm(getEncoderValue(), getArmSetpoint(), SwingArmConstants.PID_THRESHOLD);
        }
    }

    private void controlArm(double encoder, double setpoint, double threshold) {
        boolean isWithinThreshold = Math.abs(setpoint - encoder) < threshold;

        if (!isWithinThreshold) {
            setMotorSpeed(armPID.calculate(encoder, setpoint));
        } else {
            stopArmMotor();
        }
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Ensures the arm's PID setpoint doesn't go above the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed() {
        stopArmMotor();
        armPID.setSetpoint(getEncoderValue());

        if (getArmSetpoint() < getEncoderValue()) {
            setMotorSpeed(armPID.calculate(getEncoderValue(), getArmSetpoint()));
        }
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder.</li>
     *  <li>Ensures the arm's PID setpoint doesn't go below the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        resetArmEncoder();
        stopArmMotor();

        if (getArmSetpoint() > 0) {
            setMotorSpeed(armPID.calculate(getEncoderValue(), getArmSetpoint()));
        }
    }


    /** ----- Encoder and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link SwingArmSubsystem#topLimitSwitch} is pressed.
     */
    private boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link SwingArmSubsystem#bottomLimitSwitch} is pressed.
     */
    private boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speed for the arm motor.
     * @param speed The desired speed of the motor.
     */
    private void setMotorSpeed(double speed) {
        // TODO: Determine if inverting is necessary.
        swingArmMotor.set(speed);
    }

    /**
     * Stops the arm motor.
     * <p>Stops the motor to prevent any movement.</p>
     */
    private void stopArmMotor() {
        swingArmMotor.stopMotor();
    }

    /**
     * Normalize the encoder reading to a positive value.
     * @return The positive encoder reading.
     */
    private double getEncoderValue() {
        return -swingArmEncoder.get();
    }

    /**
     * Retrieves the current arm PID setpoint.
     * @return The current setpoint of the arm PID controller.
     */
    private double getArmSetpoint() {
        return armPID.getSetpoint();
    }

    /** Resets the arm encoder. */
    private void resetArmEncoder() {
        swingArmEncoder.reset();
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Returns a command to move the arm up.
     * @return The command to move the arm up.
     */
    public Command moveArmUpCommand() {
        return run (
            () -> manageManualLimitSwitches(isTopLimitSwitchPressed(), SwingArmConstants.MANUAL_CONTROL_SPEED)
        );
    }

    /**
     * Returns a command to move the arm down.
     * @return The command to move the arm down.
     */
    public Command moveArmDownCommand() {
        return run(
            () -> manageManualLimitSwitches(isBottomLimitSwitchPressed(), -SwingArmConstants.MANUAL_CONTROL_SPEED)
        );
    }

    public Command stopArmMotorCommand() {
        return runOnce(
            () -> stopArmMotor()
        );
    }

    private void manageManualLimitSwitches(boolean limitswitchPressed, double speed) {
        if (!limitswitchPressed) {
            setMotorSpeed(speed);
        } else {
            stopArmMotor();
        }
    }
}
