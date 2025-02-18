package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorState extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorStates elevatorState;

    public SetElevatorState(ElevatorSubsystem subsystem, ElevatorStates state) {
        this.elevatorSubsystem = subsystem;
        this.elevatorState = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorState(elevatorState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
