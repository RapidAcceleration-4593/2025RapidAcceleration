package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    
    private final ProfiledPIDController armPID = new ProfiledPIDController(ArmConstants.ARM_PID.kP,
                                                                           ArmConstants.ARM_PID.kI,
                                                                           ArmConstants.ARM_PID.kD,
                                                                           new TrapezoidProfile.Constraints(
                                                                                ArmConstants.MAX_VELOCITY,
                                                                                ArmConstants.MAX_ACCELERATION));

    private final ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.FEEDFORWARD_kS, ArmConstants.FEEDFORWARD_kV, ArmConstants.FEEDFORWARD_kA); // Might need kG after kS.

    private final double[] setpoints = {0, 0, 0, 0, 0}; // TODO: Determine setpoint values.

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
        double target = getArmStates(state);
        armPID.setGoal(target);
        SmartDashboard.putNumber("A-Setpoint", target);
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
            controlArm();
        }
    }

    /** Controls the arm based on the encoder feedback and setpoint. */
    private void controlArm() {
        boolean atSetpoint = armPID.atGoal();

        if (atSetpoint) {
            // Stop the motors and hold the elevator in place.
            stopArmMotor();
        } else {
            // Use PID control to adjust the motor output.
            double pidOutput = armPID.calculate(getEncoderValue());
            double ffOutput = armFeedforward.calculate(Math.toRadians(getEncoderValue()), armPID.getSetpoint().velocity);
            setMotorSpeed(pidOutput + ffOutput);
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
        armPID.reset(getEncoderValue());
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
        armPID.reset(0);
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

    /** Resets the arm encoder. */
    private void resetArmEncoder() {
        SmartDashboard.putNumber("A-Encoder", 0);
        armEncoder.reset();
    }
}
