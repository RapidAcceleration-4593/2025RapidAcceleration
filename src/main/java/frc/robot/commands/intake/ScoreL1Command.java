package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreL1Command extends SequentialCommandGroup {
    
    public ScoreL1Command(IntakeSubsystem intakeSubsystem) {
        addCommands(
            // Drive To Closest Reef, Backward.
            new SetIntakeState(intakeSubsystem, IntakeStates.L1),
            new RunIntakeCommand(intakeSubsystem, true)
        );
    }
}
