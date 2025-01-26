package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MaintainElevatorLevel extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;

    public MaintainElevatorLevel(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void execute() {
        elevatorSubsystem.controlElevatorState();
    }
}
