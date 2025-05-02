package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates.ControlDirections;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class ManualIntakeCommand extends Command {

    private final IntakeDeploySubsystem intakeDeploySubsystem;
    private final ControlDirections direction;

    public ManualIntakeCommand(IntakeDeploySubsystem subsystem, ControlDirections direction) {
        this.intakeDeploySubsystem = subsystem;
        this.direction = direction;
        addRequirements(intakeDeploySubsystem);
    }

    @Override
    public void execute() {
        if (!intakeDeploySubsystem.isManualControlEnabled()) {
            return;
        }

        double speed = (direction == ControlDirections.UP)
            ? -IntakeConstants.DEPLOY_SPEED
            : IntakeConstants.DEPLOY_SPEED;

            intakeDeploySubsystem.setMotorSpeeds(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeDeploySubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
