package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class ToggleIntakeManualControl extends Command {
    
    private final IntakeDeploySubsystem intakeDeploySubsystem;

    public ToggleIntakeManualControl(IntakeDeploySubsystem subsystem) {
        this.intakeDeploySubsystem = subsystem;
    }

    @Override
    public void initialize() {
        intakeDeploySubsystem.setManualControl(!intakeDeploySubsystem.isManualControlEnabled());
        intakeDeploySubsystem.setSetpoint(intakeDeploySubsystem.getEncoderValue());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
