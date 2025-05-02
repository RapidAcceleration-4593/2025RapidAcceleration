package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.subsystems.utils.ControlSubsystem;

public class ElevatorSubsystem extends ControlSubsystem<ElevatorStates> {
    
    private final SparkMax leaderMotor = ElevatorConstants.RIGHT_ELEVATOR_MOTOR;
    private final SparkMax followerMotor = ElevatorConstants.LEFT_ELEVATOR_MOTOR;

    private final DigitalInput topLimitSwitch = ElevatorConstants.TOP_LIMIT_SWITCH;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.BOTTOM_LIMIT_SWITCH;

    private final Encoder encoder = ElevatorConstants.ELEVATOR_ENCODER;

    private static final double[] SETPOINTS = {-500, 3750, 12800};

    private final SparkMaxConfig brakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    public ElevatorSubsystem() {
        super(
            new ProfiledPIDController(
                ElevatorPIDConstants.ELEVATOR_PID.kP,
                ElevatorPIDConstants.ELEVATOR_PID.kI,
                ElevatorPIDConstants.ELEVATOR_PID.kD,
                new TrapezoidProfile.Constraints(
                    ElevatorPIDConstants.MAX_VELOCITY,
                    ElevatorPIDConstants.MAX_ACCELERATION
                )
            )
        );

        brakeConfig.idleMode(IdleMode.kBrake);
        followerConfig.follow(leaderMotor, true);

        leaderMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.setTolerance(ElevatorPIDConstants.TOLERANCE);
        controller.reset(0);
    }

    @Override
    public double getStateSetpoint(ElevatorStates state) {
        return switch(state) {
            case BOTTOM -> SETPOINTS[0];
            case PICKUP -> SETPOINTS[1];
            case TOP -> SETPOINTS[2];
            default -> throw new Error("Passed in an ElevatorState that does not have an associated setpoint!");
        };
    }

    @Override
    public ElevatorStates getCurrentState() {
        if (getEncoderValue() <= SETPOINTS[0] + 500) {
            return ElevatorStates.BOTTOM;
        } else if (getEncoderValue() <= SETPOINTS[1] + 500) {
            return ElevatorStates.PICKUP;
        } else {
            return ElevatorStates.TOP;
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
        return bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speeds for the elevator.
     * @param speed The speed of the elevator motors.
     */
    public void setMotorSpeeds(double speed) {
        leaderMotor.set(speed);
    }

    /**
     * Stops the elevator motors.
     */
    public void stopMotors() {
        leaderMotor.stopMotor();
    }

    @Override
    public double getEncoderValue() {
        return -encoder.get();
    }

    @Override
    public void resetEncoder() {
        encoder.reset();
    }

    @Override
    protected void updateValues() {
        SmartDashboard.putBoolean("E-TopLS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("E-BotLS", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("E-Encoder", getEncoderValue());
        SmartDashboard.putNumber("E-Setpoint", getSetpoint());
        SmartDashboard.putString("E-State", getCurrentState().toString());
    }
}
