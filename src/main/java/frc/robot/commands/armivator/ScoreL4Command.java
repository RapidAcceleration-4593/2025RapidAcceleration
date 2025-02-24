package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreL4Command extends SequentialCommandGroup {

    public ScoreL4Command(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        
        ElevatorStates currentElevatorState = elevatorSubsystem.getCurrentElevatorState();
        
        switch (currentElevatorState) {
            case BOTTOM:
                addCommands(
                    elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                    Commands.parallel(
                        elevatorSubsystem.GoToStateCommand(ElevatorStates.TOP),
                        armSubsystem.GoToStateCommand(ArmStates.TOP)
                    )
                );
                break;
            
            case PICKUP:
                addCommands(
                    Commands.parallel(
                        elevatorSubsystem.GoToStateCommand(ElevatorStates.TOP),
                        armSubsystem.GoToStateCommand(ArmStates.TOP)
                    )
                );
                break;
            
            case TOP:
                addCommands(
                    armSubsystem.GoToStateCommand(ArmStates.TOP)
                );
                break;
            
            default:
                addCommands(
                    elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                    Commands.parallel(
                        elevatorSubsystem.GoToStateCommand(ElevatorStates.TOP),
                        armSubsystem.GoToStateCommand(ArmStates.TOP)
                    )
                );
                break;
        }
    }
}
