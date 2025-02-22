package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ControlElevatorState extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final boolean usePID;

    public ControlElevatorState(ElevatorSubsystem subsystem, boolean usePID) {
        this.elevatorSubsystem = subsystem;
        this.usePID = usePID;
        addRequirements(subsystem);
    }
    
    @Override
    public void execute() {
        elevatorSubsystem.controlElevatorState(usePID);
    }
}
