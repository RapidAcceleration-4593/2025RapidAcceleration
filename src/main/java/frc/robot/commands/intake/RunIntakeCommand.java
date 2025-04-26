package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class RunIntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final boolean isInverted;

    public RunIntakeCommand(IntakeSubsystem subsystem, boolean isInverted) {
        this.intakeSubsystem = subsystem;
        this.isInverted = isInverted;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(
            isInverted ? -IntakeConstants.OUTTAKE_SPEED : IntakeConstants.INTAKE_SPEED,
            isInverted ? -IntakeConstants.OUTTAKE_SPEED : IntakeConstants.INTAKE_SPEED
        );
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
