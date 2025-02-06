package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwingArmConstants.SwingArmStates;
import frc.robot.subsystems.SwingArmSubsystem;

public class SetArmSetpoint extends Command {
    
    private final SwingArmSubsystem swingArmSubsystem;
    private final SwingArmStates state;

    public SetArmSetpoint(SwingArmSubsystem swingArmSubsystem, SwingArmStates state) {
        this.swingArmSubsystem = swingArmSubsystem;
        this.state = state;
        addRequirements(swingArmSubsystem);
    }

    @Override
    public void initialize() {
        swingArmSubsystem.setArmSetpoint(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
