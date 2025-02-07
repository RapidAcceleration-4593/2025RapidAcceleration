package frc.robot.commands.arm.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmUp extends Command {
    
    private final ArmSubsystem swingArmSubsystem;

    public MoveArmUp(ArmSubsystem subsystem) {
        this.swingArmSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (swingArmSubsystem.isTopLimitSwitchPressed()) {
            swingArmSubsystem.stopArmMotor();
        } else {
            swingArmSubsystem.setMotorSpeed(ArmConstants.MANUAL_CONTROL_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swingArmSubsystem.stopArmMotor();
    }
}
