package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
        if (!handleLimitSwitchSafety(true)) {
            swingArmMotor.set(armPID.calculate(getEncoderValue(), getArmSetpoint()));
        }   
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Checks if the top or bottom limit switch is triggered, and stops the motor accordingly. It can do this in two ways:
     * <ol>
     *  <li>Control the PID setpoint and check for bad PID output that would cause the motor to drive into the limit switch.
     *  <li>Directly read the motor speed and check for a motor speed that would cause the motor to drive into the limit switch.
     * </ol>
     * This is controlled by the paramater {@link usePID}. {@link usePID} should be false if the motor speed is being directly
     * controlled and true if it is being controlled by PID.
     * @param usePID If true, this method will use method 1. Otherwise, method 2.
     * @return Whether any limit switch is pressed.
     */
    private boolean handleLimitSwitchSafety(boolean usePID) {
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            stopArmMotor();
            return true;
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed(usePID);
            return true;
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed(usePID);
            return true;
        }

        return false;
    }

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Ensures the arm's PID setpoint doesn't go above the current encoder position.</li>
     *  <li>Calculates the PID output and stops the motor if output is attempting to drive past
     *      limit switch, otherwise sets the motor speed to the PID output.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed(boolean usePID) {
        double currentPosition = getEncoderValue();
        if (getArmSetpoint() > currentPosition) {
            armPID.setSetpoint(currentPosition);
        }

        // TODO: Confirm that on the new robot positive motor speed moves the arm upwards.
        if (!usePID && getMotorSpeed() > SwingArmConstants.LS_PID_THRESHOLD) {
            swingArmMotor.stopMotor();
            return;
        }

        double pidOutput = armPID.calculate(currentPosition);
        swingArmMotor.set(pidOutput > SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
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
    private void handleBottomLimitSwitchPressed(boolean usePID) {
        resetArmEncoder();

        double currentPosition = getEncoderValue();
        if (getArmSetpoint() < currentPosition) {
            armPID.setSetpoint(currentPosition);
        }
        
        // TODO: Confirm that on the new robot negative motor speed moves the arm downwards.
        if (!usePID && getMotorSpeed() < -SwingArmConstants.LS_PID_THRESHOLD) {
            swingArmMotor.stopMotor();
            return;
        }

        double pidOutput = armPID.calculate(currentPosition);
        swingArmMotor.set(pidOutput < -SwingArmConstants.LS_PID_THRESHOLD ? 0 : pidOutput);
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
     * Retrieves the current speed of the motor. 
     * @returns The current set speed of the {@link #swingArmMotor}.
    */
    private double getMotorSpeed() {
        return swingArmMotor.get();
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
            () -> setMotorSpeed(SwingArmConstants.MANUAL_CONTROL_SPEED)
        );
    }

    /**
     * Returns a command to move the arm down.
     * @return The command to move the arm down.
     */
    public Command moveArmDownCommand() {
        return run(
            () -> setMotorSpeed(-SwingArmConstants.MANUAL_CONTROL_SPEED)
        );
    }
}
