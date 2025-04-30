package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeL1Command extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public IntakeL1Command(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SPEED);
        intakeSubsystem.setControlState(IntakeStates.OUT);
    }

    @Override
    public void execute() {
        intakeSubsystem.controlOutput();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setControlState(IntakeStates.L1);
        // intakeSubsystem.controlOutput();
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
