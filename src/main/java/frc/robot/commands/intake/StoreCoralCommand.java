package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StoreCoralCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public StoreCoralCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.storeCoral();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
