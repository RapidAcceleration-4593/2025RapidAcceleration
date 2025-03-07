package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotStates.Arm.ArmDirections;
import frc.robot.Constants.RobotStates.Elevator.ElevatorDirections;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorDirections direction;

    public ManualElevatorCommand(ElevatorSubsystem subsystem, ElevatorDirections direction) {
        this.elevatorSubsystem = subsystem;
        this.direction = direction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        if (!elevatorSubsystem.isManualControlEnabled())
            return;

        double speed = (direction == ElevatorDirections.UP)
        ? ElevatorConstants.CONTROL_SPEED
        : -ElevatorConstants.CONTROL_SPEED;

        //TODO: Add resetEncoder when bottom LS is hit.

        if ((elevatorSubsystem.isTopLimitSwitchPressed() && speed > 0) ||
            (elevatorSubsystem.isBottomLimitSwitchPressed() && speed < 0)) {
            elevatorSubsystem.stopMotors();
            return;
        }
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
}
