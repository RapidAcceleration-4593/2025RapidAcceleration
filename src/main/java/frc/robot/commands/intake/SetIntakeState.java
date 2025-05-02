package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class SetIntakeState extends Command {
    
    private final IntakeDeploySubsystem intakeDeploySubsystem;
    private final IntakeStates intakeState;

    public SetIntakeState(IntakeDeploySubsystem subsystem, IntakeStates state) {
        this.intakeDeploySubsystem = subsystem;
        this.intakeState = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeDeploySubsystem.setControlState(intakeState);
    }

    @Override
    public void execute() {
        intakeDeploySubsystem.controlOutput();
    }

    @Override
    public boolean isFinished() {
        return intakeDeploySubsystem.atSetpoint();
    }
}
