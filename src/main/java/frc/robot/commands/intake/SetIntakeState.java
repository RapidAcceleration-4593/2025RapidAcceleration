package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeState extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeStates intakeState;

    public SetIntakeState(IntakeSubsystem subsystem, IntakeStates state) {
        this.intakeSubsystem = subsystem;
        this.intakeState = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setControlState(intakeState);
    }

    @Override
    public void execute() {
        intakeSubsystem.controlOutput();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.atSetpoint();
    }
}
