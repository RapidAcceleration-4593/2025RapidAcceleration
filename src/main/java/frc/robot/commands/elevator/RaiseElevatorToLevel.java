package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.ElevatorSubsystem;

public class RaiseElevatorToLevel extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorLevel level;

    public RaiseElevatorToLevel(ElevatorSubsystem elevatorSubsystem, ElevatorLevel level) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.level = level;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.raiseElevatorToLevel(level);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
