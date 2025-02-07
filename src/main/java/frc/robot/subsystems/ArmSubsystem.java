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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor = ArmConstants.armMotor;
    private final Encoder armEncoder = ArmConstants.armEncoder;

    private final DigitalInput topLimitSwitch = ArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;
    
    private final PIDController armPID = new PIDController(ArmConstants.ARM_PID.kP,
                                                           ArmConstants.ARM_PID.kI,
                                                           ArmConstants.ARM_PID.kD);

    private final double[] setpoints = {0, 0, 0, 0, 0}; // TODO: Determine setpoint values.

    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the SwingArmSubsystem class.
     * Initializes the motor and encoder configuration.
     */
    public ArmSubsystem() {
        config.idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Arm State Management ----- */

    /**
     * Retrieves the setpoint for the specified SwingArmStates.
     * @param state The desired arm position.
     * @return The {@link ArmSubsystem#setpoints} value corresponding to the state.
     */
    private double getArmStates(ArmConstants.ArmStates state) {
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
    public void setArmSetpoint(ArmConstants.ArmStates state) {
        armPID.setSetpoint(getArmStates(state));
        SmartDashboard.putNumber("A-Setpoint", getArmSetpoint());
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
            controlArm(getEncoderValue(), getArmSetpoint(), ArmConstants.PID_THRESHOLD);
        }
    }

    /**
     * Controls the arm based on the encoder feedback and setpoint.
     * @param encoder The current encoder reading.
     * @param setpoint The desired setpoint for the arm.
     * @param threshold The threshold for the PID controller.
     */
    private void controlArm(double encoder, double setpoint, double threshold) {
        boolean isWithinThreshold = Math.abs(setpoint - encoder) < threshold;
        SmartDashboard.putBoolean("A-WithinThreshold", isWithinThreshold);

        if (isWithinThreshold) {
            // Stop the motors and hold the elevator in place.
            stopArmMotor();
        } else {
            // Use PID control to adjust the motor output.
            setMotorSpeed(armPID.calculate(encoder, setpoint));
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
        // Reset the encoder and stop the motor.
        stopArmMotor();
        armPID.setSetpoint(getEncoderValue());

        // Ensure the arm doesn't go above the current encoder position.
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
        // Reset the encoder and stop the motor.
        resetArmEncoder();
        stopArmMotor();

        // Ensure the arm doesn't go below the current encoder position.
        if (getArmSetpoint() > 0) {
            setMotorSpeed(armPID.calculate(getEncoderValue(), getArmSetpoint()));
        }
    }


    /** ----- Encoder and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ArmSubsystem#topLimitSwitch} is pressed.
     */
    public boolean isTopLimitSwitchPressed() {
        return logAndReturn("A-TopLimitSwitch", !topLimitSwitch.get());
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ArmSubsystem#bottomLimitSwitch} is pressed.
     */
    public boolean isBottomLimitSwitchPressed() {
        return logAndReturn("A-BotLimitSwitch", !bottomLimitSwitch.get());
    }

    /**
     * Sets the motor speed for the arm motor.
     * @param speed The desired speed of the motor.
     */
    public void setMotorSpeed(double speed) {
        armMotor.set(speed);
    }

    /**
     * Stops the arm motor.
     * <p>Stops the motor to prevent any movement.</p>
     */
    public void stopArmMotor() {
        armMotor.stopMotor();
    }

    /**
     * Retrieves the current encoder value of the arm.
     * @return The current encoder value of the arm.
     */
    private double getEncoderValue() {
        return logAndReturn("A-Encoder", -armEncoder.get());
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
        armEncoder.reset();
        SmartDashboard.putNumber("A-Encoder", 0);
    }

    /**
     * Logs the value to SmartDashboard and returns it.
     * @param <X> The type of value to log.
     * @param key The key to log the value under.
     * @param value The value to log.
     * @return The value that was logged.
     */
    private <X> X logAndReturn(String key, X value) {
        if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof Double) {
            SmartDashboard.putNumber(key, (Double) value);
        }

        return value;
    }
}
