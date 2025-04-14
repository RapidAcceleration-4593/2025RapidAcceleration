package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeManualControl extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public ToggleIntakeManualControl(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setManualControl(!intakeSubsystem.isManualControlEnabled());
        intakeSubsystem.setSetpoint(intakeSubsystem.getEncoderValue());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
