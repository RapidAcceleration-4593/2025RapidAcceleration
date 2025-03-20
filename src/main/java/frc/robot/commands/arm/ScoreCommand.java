package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreCommand extends SequentialCommandGroup {
    
    public ScoreCommand(ArmSubsystem armSubsystem, SwerveSubsystem drivebase) {
        addCommands(
            new AdjustArmCommand(armSubsystem, -ArmConstants.PLACE_ROTATION_AMOUNT).withTimeout(ArmTravelTime.SCORE),
            drivebase.driveToDistance(-0.75, 2.5, 2.25).withTimeout(0.8)
        );
    }
}
