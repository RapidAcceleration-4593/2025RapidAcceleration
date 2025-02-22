package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ControlArmState extends Command {
    
    private final ArmSubsystem armSubsystem;
    private final boolean usePID;

    public ControlArmState(ArmSubsystem subsystem, boolean usePID) {
        this.armSubsystem = subsystem;
        this.usePID = usePID;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        armSubsystem.controlArmState(usePID);
    }
}
