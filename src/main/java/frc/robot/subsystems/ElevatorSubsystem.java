package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.subsystems.control.ControlSubsystem;

public class ElevatorSubsystem extends ControlSubsystem<ElevatorStates> {
    
    private final SparkMax leaderMotor = ElevatorConstants.rightElevatorMotor;
    private final SparkMax followerMotor = ElevatorConstants.leftElevatorMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final Encoder encoder = ElevatorConstants.elevatorEncoder;

    private static final double[] SETPOINTS = {-500, 3750, 12550};

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

        applyBrakeConfig(List.of(leaderMotor, followerMotor));
        applyFollowerConfig(leaderMotor, followerMotor, true);

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

    @Override
    public void controlOutput() {
        double output = controller.calculate(getEncoderValue(), getSetpoint());

        if (atSetpoint()) {
            stopMotors();
        } else {
            setMotorSpeeds(output);
        }
    }

    private void handleTopLimitSwitchPressed() {
        if (getSetpoint() >= getEncoderValue()) {
            stopMotors();
            resetSetpoint(getEncoderValue());
        } else {
            controlOutput();
        }
    }

    private void handleBottomLimitSwitchPressed() {
        resetEncoder();

        if (getSetpoint() <= 0) {
            stopMotors();
            resetSetpoint(0);
        } else {
            controlOutput();
        }
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return bottomLimitSwitch.get();
    }

    public void setMotorSpeeds(double speed) {
        leaderMotor.set(speed);
    }

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
