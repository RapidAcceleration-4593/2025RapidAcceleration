package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotStates.ControlDirections;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends Command {
    
    private final ArmSubsystem armSubsystem;
    private final ControlDirections direction;

    public ManualArmCommand(ArmSubsystem subsystem, ControlDirections direction) {
        this.armSubsystem = subsystem;
        this.direction = direction;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        if (!armSubsystem.isManualControlEnabled()) {
            return;
        }

        if (armSubsystem.isBottomLimitSwitchPressed()) {
            armSubsystem.resetEncoder();
        }

        if (isLimitSwitchPressed()) {
            armSubsystem.stopMotors();
            return;
        }

        double speed = (direction == ControlDirections.UP)
            ? ArmConstants.CONTROL_SPEED
            : -ArmConstants.CONTROL_SPEED;

        armSubsystem.setMotorSpeeds(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isLimitSwitchPressed() {
        return (direction == ControlDirections.UP)
            ? armSubsystem.isTopLimitSwitchPressed()
            : armSubsystem.isBottomLimitSwitchPressed();
    }
}
