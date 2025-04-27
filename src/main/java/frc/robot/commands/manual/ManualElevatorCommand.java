package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStates.ControlDirections;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ControlDirections direction;

    public ManualElevatorCommand(ElevatorSubsystem subsystem, ControlDirections direction) {
        this.elevatorSubsystem = subsystem;
        this.direction = direction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        if (!elevatorSubsystem.isManualControlEnabled()) {
            return;
        }

        if (elevatorSubsystem.isBottomLimitSwitchPressed()) {
            elevatorSubsystem.resetEncoder();
        }
        
        if (isLimitSwitchPressed()) {
            elevatorSubsystem.stopMotors();
            return;
        }

        double speed = (direction == ControlDirections.UP)
            ? ElevatorConstants.CONTROL_SPEED
            : -ElevatorConstants.CONTROL_SPEED;

        elevatorSubsystem.setMotorSpeeds(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isLimitSwitchPressed() {
        return (direction == ControlDirections.UP)
            ? elevatorSubsystem.isTopLimitSwitchPressed()
            : elevatorSubsystem.isBottomLimitSwitchPressed();
    }
}
