package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreCoralCommand extends Command {
    
    private final ArmSubsystem armSubsystem;
    private int scoreAmount;

    public ScoreCoralCommand(ArmSubsystem subsystem, int scoreAmount) {
        this.armSubsystem = subsystem;
        this.scoreAmount = scoreAmount;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(armSubsystem.getSetpoint() + scoreAmount);
    }

    @Override
    public void execute() {
        armSubsystem.controlArmState();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotor();
    }
}
