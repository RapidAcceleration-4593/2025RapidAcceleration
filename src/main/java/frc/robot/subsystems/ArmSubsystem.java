package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.subsystems.control.ControlSubsystem;

public class ArmSubsystem extends ControlSubsystem<ArmStates> {

    private final SparkMax motor = ArmConstants.armMotor;
    private final Encoder encoder = ArmConstants.armEncoder;

    private final DigitalInput topLimitSwitch = ArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;

    private static final double[] SETPOINTS = {-20, 620, 875};

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

        applyBrakeConfig(List.of(motor));

        controller.setTolerance(ArmPIDConstants.TOLERANCE);
        controller.reset(0);
    }

    @Override
    public double getStateSetpoint(ArmStates state) {
        return switch (state) {
            case BOTTOM -> SETPOINTS[0];
            case L2 -> SETPOINTS[1];
            case TOP -> SETPOINTS[2];
            default -> throw new Error("Passed in an ArmState that does not have an associated setpoint!");
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
            stopMotor();
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
            stopMotor();
        } else {
            setMotorSpeed(output);
        }
    }

    private void handleTopLimitSwitchPressed() {
        if (getSetpoint() >= getEncoderValue()) {
            stopMotor();
            resetSetpoint(getEncoderValue());
        } else {
            controlOutput();
        }
    }

    private void handleBottomLimitSwitchPressed() {
        resetEncoder();
                    
        if (getSetpoint() <= 0) {
            stopMotor();
            resetSetpoint(0);
        } else {
            controlOutput();
        }
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
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
