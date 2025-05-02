package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class RunIntakeCommand extends Command {
    
    private final IntakeFeederSubsystem intakeFeederSubsystem;
    private final boolean isInverted;

    public RunIntakeCommand(IntakeFeederSubsystem intake, boolean isInverted) {
        this.intakeFeederSubsystem = intake;
        this.isInverted = isInverted;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intakeFeederSubsystem.setIntakeSpeed(
            isInverted ? -IntakeConstants.OUTTAKE_SPEED : IntakeConstants.INTAKE_SPEED,
            isInverted ? -IntakeConstants.OUTTAKE_SPEED : IntakeConstants.INTAKE_SPEED
        );
    }

    @Override
    public void end(boolean interrupted) {
        intakeFeederSubsystem.stopIntake();
    }
}
