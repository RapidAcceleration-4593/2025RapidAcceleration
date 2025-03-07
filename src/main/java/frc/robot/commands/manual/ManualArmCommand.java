package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotStates.Arm.ArmDirections;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends Command {
    
    private final ArmSubsystem armSubsystem;
    private final ArmDirections direction;

    public ManualArmCommand(ArmSubsystem subsystem, ArmDirections direction) {
        this.armSubsystem = subsystem;
        this.direction = direction;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        if (!armSubsystem.isManualControlEnabled())
            return;

        double speed = (direction == ArmDirections.UP)
        ? ArmConstants.CONTROL_SPEED
        : -ArmConstants.CONTROL_SPEED;
        
        //TODO: Add resetEncoder when bottom LS is hit.

        if ((armSubsystem.isTopLimitSwitchPressed() && speed > 0) ||
            (armSubsystem.isBottomLimitSwitchPressed() && speed < 0)) {
            armSubsystem.stopMotor();
            return;
        }
        armSubsystem.setMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
