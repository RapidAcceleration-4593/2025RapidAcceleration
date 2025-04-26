package frc.robot.subsystems.utils;

import java.util.List;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ControlSubsystem<ControlStates extends Enum<ControlStates>> extends SubsystemBase {
    
    protected final ProfiledPIDController controller;

    protected ControlStates currentState;
    protected boolean isManualControlEnabled = false;

    protected SparkMaxConfig brakeConfig = new SparkMaxConfig();
    protected SparkMaxConfig followerConfig = new SparkMaxConfig();

    public ControlSubsystem(ProfiledPIDController pidController) {
        this.controller = pidController;
    }

    public abstract double getStateSetpoint(ControlStates state);
    public abstract ControlStates getCurrentState();

    public abstract void controlStates();

    public abstract void stopMotors();
    public abstract void setMotorSpeeds(double speed);

    public abstract double getEncoderValue();
    public abstract void resetEncoder();

    protected abstract void updateValues();

    public void setControlState(ControlStates state) {
        this.currentState = state;
        setSetpoint(getStateSetpoint(state));
    }

    public void controlOutput() {
        double output = controller.calculate(getEncoderValue(), getSetpoint());
    
        if (atSetpoint()) {
            stopMotors();
        } else {
            setMotorSpeeds(output);
        }
    }
    
    public void setSetpoint(double setpoint) {
        controller.setGoal(setpoint);
    }

    public double getSetpoint() {
        return controller.getGoal().position;
    }

    public boolean atSetpoint() {
        return controller.atGoal();
    }

    public void resetSetpoint(double setpoint) {
        controller.setGoal(setpoint);
        controller.reset(setpoint);
    }

    public void setManualControl(boolean isEnabled) {
        isManualControlEnabled = isEnabled;
    }

    public boolean isManualControlEnabled() {
        return isManualControlEnabled;
    }

    public void applyBrakeConfig(List<SparkMax> motors) {
        brakeConfig.idleMode(IdleMode.kBrake);
        for (SparkMax motor : motors) {
            motor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}