package frc.robot.commands.intake.left;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class RunLeftIntake extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public RunLeftIntake(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (intakeSubsystem.isLeftExtended()) {
            intakeSubsystem.setLeftState(IntakeStates.EXTENDED_RUNNING);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setLeftState(IntakeStates.EXTENDED_STOPPED);
    }
}
