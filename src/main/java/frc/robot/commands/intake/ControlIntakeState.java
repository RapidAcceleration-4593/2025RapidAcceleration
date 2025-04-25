package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ControlIntakeState extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public ControlIntakeState(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.controlStates();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
