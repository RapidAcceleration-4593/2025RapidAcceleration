package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates.ControlDirections;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final ControlDirections direction;

    public ManualIntakeCommand(IntakeSubsystem subsystem, ControlDirections direction) {
        this.intakeSubsystem = subsystem;
        this.direction = direction;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!intakeSubsystem.isManualControlEnabled()) {
            return;
        }

        double speed = (direction == ControlDirections.UP)
            ? -IntakeConstants.DEPLOY_SPEED
            : IntakeConstants.DEPLOY_SPEED;

        intakeSubsystem.setMotorSpeeds(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
