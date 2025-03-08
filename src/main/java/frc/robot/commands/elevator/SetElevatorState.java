package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
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
        elevatorSubsystem.setTargetElevatorState(elevatorState);
    }

    @Override
    public void execute() {
        elevatorSubsystem.controlElevatorState();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
    }
}
