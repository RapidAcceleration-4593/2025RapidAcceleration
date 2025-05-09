package frc.robot.commands.arm.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ControlArmState extends Command {
    
    private final ArmSubsystem armSubsystem;

    public ControlArmState(ArmSubsystem subsystem) {
        this.armSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        armSubsystem.controlStates();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
