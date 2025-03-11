package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AdjustArmCommand extends Command {
    
    private final ArmSubsystem armSubsystem;
    private int offset;

    public AdjustArmCommand(ArmSubsystem subsystem, int offset) {
        this.armSubsystem = subsystem;
        this.offset = offset;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(armSubsystem.getSetpoint() + offset);
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
