package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCommand extends Command {
    
    private final ClimberSubsystem climberSubsystem;
    private final boolean climberInverted;

    public RunClimberCommand(ClimberSubsystem subsystem, boolean inverted) {
        this.climberSubsystem = subsystem;
        this.climberInverted = inverted;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.runClimber(climberInverted);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimber();
    }
}
