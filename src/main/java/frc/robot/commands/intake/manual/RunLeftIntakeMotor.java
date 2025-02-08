package frc.robot.commands.intake.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunLeftIntakeMotor extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public RunLeftIntakeMotor(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runLeftExtensionIn();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopLeftIntake();
    }
}
