package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwingArmSubsystem;

public class MaintainArmAngle extends Command {
    
    private final SwingArmSubsystem swingArmSubsystem;

    public MaintainArmAngle(SwingArmSubsystem subsystem) {
        this.swingArmSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        swingArmSubsystem.controlArmState();
    }
}
