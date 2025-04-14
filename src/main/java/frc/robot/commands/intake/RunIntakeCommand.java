package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class RunIntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final boolean intakeReversed;

    public RunIntakeCommand(IntakeSubsystem subsystem, boolean reversed) {
        this.intakeSubsystem = subsystem;
        this.intakeReversed = reversed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(
            intakeReversed ? -IntakeConstants.INTAKE_SPEED : IntakeConstants.INTAKE_SPEED,
            intakeReversed ? -IntakeConstants.INTAKE_SPEED : IntakeConstants.INTAKE_SPEED
        );
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
