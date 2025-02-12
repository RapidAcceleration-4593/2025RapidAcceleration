package frc.robot.commands.intake.manual.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunLeftIntakeReverseCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public RunLeftIntakeReverseCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runLeftIntake(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopLeftIntake();
    }
}
