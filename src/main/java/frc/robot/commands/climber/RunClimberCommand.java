package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.Climber.ClimberDirections;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCommand extends Command {
    
    private final ClimberSubsystem climberSubsystem;
    private final ClimberDirections climberDirection;

    public RunClimberCommand(ClimberSubsystem subsystem, ClimberDirections direction) {
        this.climberSubsystem = subsystem;
        this.climberDirection = direction;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.runClimber(climberDirection);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimber();
    }
}
