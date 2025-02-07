package frc.robot.commands.elevator.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorDown extends Command {

    private final ElevatorSubsystem elevatorSubsystem;

    public MoveElevatorDown(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void execute() {
        if (elevatorSubsystem.isBottomLimitSwitchPressed()) {
            elevatorSubsystem.stopElevatorMotors();
        } else {
            elevatorSubsystem.setMotorSpeeds(-ElevatorConstants.MANUAL_CONTROL_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevatorMotors();
    }
}
