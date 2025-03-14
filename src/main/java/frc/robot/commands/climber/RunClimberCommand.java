package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCommand extends Command {
    
    private final ClimberSubsystem climberSubsystem;
    private final boolean climberReversed;

    public RunClimberCommand(ClimberSubsystem subsystem, boolean reversed) {
        this.climberSubsystem = subsystem;
        this.climberReversed = reversed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.runClimber(climberReversed);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimber();
    }
}
