package frc.robot.commands.intake.right;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class RunRightIntake extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public RunRightIntake(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (intakeSubsystem.isRightExtended() && !intakeSubsystem.isRightIntakeRunning()) {
            intakeSubsystem.setRightState(IntakeStates.EXTENDED_RUNNING);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRightState(IntakeStates.EXTENDED_STOPPED);
    }
}
