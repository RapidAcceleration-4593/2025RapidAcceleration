package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MaintainArmAngle extends Command {
    
    private final ArmSubsystem armSubsystem;

    public MaintainArmAngle(ArmSubsystem subsystem) {
        this.armSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        armSubsystem.controlArmState();
    }
}
