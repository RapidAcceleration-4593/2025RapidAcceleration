package frc.robot.commands.arm.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmDown extends Command {

    private final ArmSubsystem armSubsystem;

    public MoveArmDown(ArmSubsystem subsystem) {
        this.armSubsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void execute() {
        if (armSubsystem.isBottomLimitSwitchPressed()) {
            armSubsystem.stopMotor();
        } else {
            armSubsystem.setMotorSpeed(-ArmConstants.MANUAL_CONTROL_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotor();
    }
}
