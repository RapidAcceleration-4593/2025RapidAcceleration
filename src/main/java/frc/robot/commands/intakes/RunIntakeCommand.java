package frc.robot.commands.intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeSides;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSides intakeSide;
    private final boolean intakeInverted;

    public RunIntakeCommand(IntakeSubsystem subsystem, IntakeSides side, boolean inverted) {
        this.intakeSubsystem = subsystem;
        this.intakeSide = side;
        this.intakeInverted = inverted;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntake(intakeSide, intakeInverted);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake(intakeSide);
    }
}
