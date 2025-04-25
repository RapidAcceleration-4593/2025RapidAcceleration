package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.ElevatorStates;
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
        elevatorSubsystem.setControlState(elevatorState);
    }

    @Override
    public void execute() {
        elevatorSubsystem.controlOutput();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() ||
               (elevatorSubsystem.isBottomLimitSwitchPressed() && elevatorState == ElevatorStates.BOTTOM) ||
               (elevatorSubsystem.isTopLimitSwitchPressed() && elevatorState == ElevatorStates.TOP);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
    }
}
