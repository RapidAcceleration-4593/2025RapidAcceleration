package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeL1Command extends SequentialCommandGroup {
    
    public IntakeL1Command(IntakeSubsystem intakeSubsystem) {
        addCommands(
            new SetIntakeState(intakeSubsystem, IntakeStates.OUT),
            new StoreCoralCommand(intakeSubsystem).until(intakeSubsystem::isCurrentSpikeDetected),
            new SetIntakeState(intakeSubsystem, IntakeStates.IN)
        );
    }
}
