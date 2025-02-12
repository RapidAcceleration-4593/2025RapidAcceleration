package frc.robot.commands.intake.manual.extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractRightIntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public RetractRightIntakeCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.extendRightIntake(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopRightExtension();
    }
}
