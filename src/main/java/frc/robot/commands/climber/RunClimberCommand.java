package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCommand extends Command {
    
    private final ClimberSubsystem climberSubsystem;

    public RunClimberCommand(ClimberSubsystem subsystem) {
        this.climberSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.runClimber(false);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimber();
    }
}
