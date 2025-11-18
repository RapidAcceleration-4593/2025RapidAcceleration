package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorState extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorStates state;


    public SetElevatorState(ElevatorSubsystem subsystem, ElevatorStates state) {
        this.elevatorSubsystem = subsystem;
        this.state = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setControlState(state);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint();
    }
}
