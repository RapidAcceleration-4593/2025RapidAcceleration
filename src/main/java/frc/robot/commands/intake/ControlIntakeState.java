package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class ControlIntakeState extends Command {
    
    private final IntakeDeploySubsystem intakeDeploySubsystem;

    public ControlIntakeState(IntakeDeploySubsystem subsystem) {
        this.intakeDeploySubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeDeploySubsystem.controlStates();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
