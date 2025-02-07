package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmSetpoint extends Command {
    
    private final ArmSubsystem armSubsystem;
    private final ArmStates state;

    public SetArmSetpoint(ArmSubsystem subsystem, ArmStates state) {
        this.armSubsystem = subsystem;
        this.state = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setArmSetpoint(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
