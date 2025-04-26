package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AdjustArmCommand extends Command {
    
    private final ArmSubsystem armSubsystem;
    private final double adjustment;

    public AdjustArmCommand(ArmSubsystem subsystem, double adjustment) {
        this.armSubsystem = subsystem;
        this.adjustment = adjustment;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(armSubsystem.getSetpoint() + adjustment);
    }

    @Override
    public void execute() {
        armSubsystem.controlStates();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotors();
    }
}
