package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.ArmStates;
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
        armSubsystem.setControlState(armState);
    }

    @Override
    public void execute() {
        armSubsystem.controlStates();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetpoint() ||
               (armSubsystem.isBottomLimitSwitchPressed() && armState == ArmStates.BOTTOM) ||
               (armSubsystem.isTopLimitSwitchPressed() && armState == ArmStates.TOP);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotors();
    }
}
