package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreCoralCommand extends Command {
    
    private final ArmSubsystem armSubsystem;

    public ScoreCoralCommand(ArmSubsystem subsystem) {
        this.armSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(armSubsystem.getSetpoint() - ArmConstants.PLACE_ROTATION_AMOUNT);
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
