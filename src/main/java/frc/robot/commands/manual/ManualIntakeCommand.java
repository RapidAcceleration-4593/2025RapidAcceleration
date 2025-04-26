package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates.IntakeDirections;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final IntakeDirections direction;

    public ManualIntakeCommand(IntakeSubsystem subsystem, IntakeDirections direction) {
        this.intakeSubsystem = subsystem;
        this.direction = direction;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!intakeSubsystem.isManualControlEnabled()) {
            return;
        }

        double speed = (direction == IntakeDirections.UP)
            ? IntakeConstants.DEPLOY_SPEED
            : -IntakeConstants.DEPLOY_SPEED;

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
