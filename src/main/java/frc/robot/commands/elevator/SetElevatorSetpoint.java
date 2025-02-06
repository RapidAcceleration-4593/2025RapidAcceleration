package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorSetpoint extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorStates state;

    public SetElevatorSetpoint(ElevatorSubsystem elevatorSubsystem, ElevatorStates state) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.state = state;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSetpoint(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
