package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmState extends Command {
    
    private final ArmSubsystem armSubsystem;
    private final ArmStates armState;

    public SetArmState(ArmSubsystem subsystem, ArmStates state) {
        this.armSubsystem = subsystem;
        this.armState = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setArmState(armState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
