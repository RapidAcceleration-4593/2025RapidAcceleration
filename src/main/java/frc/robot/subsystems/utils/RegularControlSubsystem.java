package frc.robot.subsystems.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class RegularControlSubsystem<T extends Enum<T>> extends SubsystemBase {
    
    protected final PIDController controller;

    protected T currentState;
    protected boolean isManualControlEnabled = false;

    protected RegularControlSubsystem(PIDController pidController) {
        this.controller = pidController;
    }

    /**
     * Retrieves the setpoint value for a provided state.
     * @param state The state to retrieve the setpoint for.
     * @return The numerical setpoint value for the provided state.
     */
    public abstract double getStateSetpoint(T state);

    /**
     * Gets the current state of the subsystem.
     * @return The current state of the subsystem.
     */
    public abstract T getCurrentState();

    /**
     * Controls the states of the subsystem based on the current state.
     */
    public abstract void controlStates();

    /** 
     * Stops the motors in the subsystem.
     */
    public abstract void stopMotors();

    /**
     * Sets the speed of the motors in the subsystem.
     */
    public abstract void setMotorSpeeds(double speed);

    /**
     * Gets the current encoder value of the subsystem.
     * @return
     */
    public abstract double getEncoderValue();

    /**
     * Resets the encoder value to zero.
     */
    public abstract void resetEncoder();

    /**
     * Updates the values of the subsystem through SmartDashboard.
     */
    protected abstract void updateValues();

    /** 
     * Sets the control state of the subsystem and updates the setpoint accordingly.
     */
    public void setControlState(T state) {
        this.currentState = state;
        setSetpoint(getStateSetpoint(state));
    }

    /** 
     * Controls the output of the motors based on the PID controller.
    */
    public void controlOutput() {
        double output = controller.calculate(getEncoderValue(), getSetpoint());
    
        if (atSetpoint()) {
            stopMotors();
        } else {
            setMotorSpeeds(output);
        }
    }
    
    /**
     * Sets the setpoint for the PID controller.
     * @param setpoint The desired setpoint for the PID controller.
     */
    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }

    /**
     * Gets the current setpoint of the PID controller.
     * @return The current setpoint of the PID controller.
     */
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    /** 
     * Checks if the PID controller is at the setpoint.
     * @return Whether the PID controller is at the setpoint.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Sets the manual control state of the subsystem.
     * @param isEnabled Whether manual control is enabled.
     */
    public void setManualControl(boolean isEnabled) {
        isManualControlEnabled = isEnabled;
    }

    /**
     * Checks if manual control is enabled.
     * @return Whether manual control is enabled.
     */
    public boolean isManualControlEnabled() {
        return isManualControlEnabled;
    }
}