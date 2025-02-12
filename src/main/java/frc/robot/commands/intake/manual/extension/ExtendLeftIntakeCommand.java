package frc.robot.commands.intake.manual.extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendLeftIntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public ExtendLeftIntakeCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.extendLeftIntake(false);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopLeftExtension();
    }
}
