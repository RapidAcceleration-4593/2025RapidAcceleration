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
import frc.robot.Constants.ArmConstants.ArmStates;

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
    private double getArmStates(ArmStates state) {
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
    public void setArmSetpoint(ArmStates state) {
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
        // Stop the motor and set current position as new setpoint.
        stopArmMotor();
        armPID.setSetpoint(getEncoderValue());
        
        if (getArmSetpoint() < getEncoderValue() - ArmConstants.PID_THRESHOLD) {
            // Ensure the arm doesn't go above the current encoder position.
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
        
        if (getArmSetpoint() > ArmConstants.PID_THRESHOLD) {
            // Ensure the arm doesn't go below the current encoder position.
            setMotorSpeed(armPID.calculate(getEncoderValue(), getArmSetpoint()));
        }
    }


    /** ----- Encoder and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ArmSubsystem#topLimitSwitch} is pressed.
     */
    public boolean isTopLimitSwitchPressed() {
        SmartDashboard.putBoolean("A-TopLS", !topLimitSwitch.get());
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ArmSubsystem#bottomLimitSwitch} is pressed.
     */
    public boolean isBottomLimitSwitchPressed() {
        SmartDashboard.putBoolean("A-BotLS", !bottomLimitSwitch.get());
        return !bottomLimitSwitch.get();
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
        SmartDashboard.putNumber("A-Encoder", -armEncoder.get());
        return -armEncoder.get();
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
        SmartDashboard.putNumber("A-Encoder", 0);
        armEncoder.reset();
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Places the coral by rotating the arm to the desired position.
     * <p>Rotates the arm to the place position by subtracting the rotation amount from the setpoint.</p>
     */
    public void placeCoralCommand() {
        boolean atSetpoint = Math.abs(getArmSetpoint() - getEncoderValue()) < ArmConstants.PID_THRESHOLD;
        boolean isBottomState = getArmSetpoint() == setpoints[0];

        if (atSetpoint && !isBottomState) {
            armPID.setSetpoint(getArmSetpoint() - ArmConstants.PLACE_ROTATION_AMOUNT);
        }
    }
}
