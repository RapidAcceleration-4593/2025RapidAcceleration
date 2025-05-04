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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.subsystems.utils.ProfiledControlSubsystem;

public class ArmSubsystem extends ProfiledControlSubsystem<ArmStates> {

    private static final SparkMax motor = ArmConstants.ARM_MOTOR;
    private static final Encoder encoder = ArmConstants.ARM_ENCODER;

    private static final DigitalInput topLimitSwitch = ArmConstants.TOP_LIMIT_SWITCH;
    private static final DigitalInput bottomLimitSwitch = ArmConstants.BOTTOM_LIMIT_SWITCH;

    private static final double[] SETPOINTS = {-20, 425, 550, 620, 875};

    private final SparkMaxConfig brakeConfig = new SparkMaxConfig();

    public ArmSubsystem() {
        super(
            new ProfiledPIDController(
                ArmPIDConstants.ARM_PID.kP,
                ArmPIDConstants.ARM_PID.kI,
                ArmPIDConstants.ARM_PID.kD,
                new TrapezoidProfile.Constraints(
                    ArmPIDConstants.MAX_VELOCITY,
                    ArmPIDConstants.MAX_ACCELERATION
                )
            )
        );

        brakeConfig.idleMode(IdleMode.kBrake);
        motor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.setTolerance(ArmPIDConstants.TOLERANCE);
        controller.reset(0);
    }

    @Override
    public double getStateSetpoint(ArmStates state) {
        return switch (state) {
            case BOTTOM -> SETPOINTS[0];
            case LOW_ALGAE -> SETPOINTS[1];
            case HIGH_ALGAE -> SETPOINTS[2];
            case L2 -> SETPOINTS[3];
            case TOP -> SETPOINTS[4];
            default -> throw new IllegalStateException("Passed in an ArmState that does not have an associated setpoint!");
        };
    }

    @Override
    public ArmStates getCurrentState() {
        if (getEncoderValue() <= SETPOINTS[0] + 100) {
            return ArmStates.BOTTOM;
        } else if (getEncoderValue() <= SETPOINTS[1] + 100) {
            return ArmStates.L2;
        } else {
            return ArmStates.TOP;
        }
    }

    @Override
    public void controlStates() {
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
            controlOutput();
        }
    }

    /**
     * Handles the logic when the top limit switch is pressed.
     * <ol>
     * <li>Stops the motors if the setpoint is greater than the encoder value.</li>
     * <li>Sets the setpoint to the encoder value.</li>
     * </ol>
     */
    private void handleTopLimitSwitchPressed() {
        if (getSetpoint() >= getEncoderValue()) {
            stopMotors();
            setSetpoint(getEncoderValue());
        } else {
            controlOutput();
        }
    }

    /**
     * Handles the logic when the bottom limit switch is pressed.
     * <ol>
     * <li>Resets the encoder.</li>
     * <li>Stops the motors if the setpoint is less than zero.</li>
     * <li>Sets the setpoint to zero.</li>
     * </ol>
     */
    private void handleBottomLimitSwitchPressed() {
        resetEncoder();
                    
        if (getSetpoint() <= 0) {
            stopMotors();
            setSetpoint(0);
        } else {
            controlOutput();
        }
    }

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether the top limit switch is pressed.
     */
    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether the bottom limit switch is pressed.
     */
    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speeds for the arm.
     * @param speed The speed of the arm motor.
     */
    public void setMotorSpeeds(double speed) {
        motor.set(speed);
    }

    /**
     * Stops the arm motors.
     */
    public void stopMotors() {
        motor.stopMotor();
    }

    @Override
    public double getEncoderValue() {
        return encoder.get();
    }

    @Override
    public void resetEncoder() {
        encoder.reset();
    }

    @Override
    protected void updateValues() {
        SmartDashboard.putBoolean("A-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("A-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("A-Encoder", getEncoderValue());
        SmartDashboard.putNumber("A-Setpoint", getSetpoint());
        SmartDashboard.putString("A-State", getCurrentState().toString());
    }
}
