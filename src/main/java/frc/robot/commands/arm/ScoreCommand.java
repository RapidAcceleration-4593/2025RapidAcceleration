package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.commands.drivebase.DriveToDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreCommand extends SequentialCommandGroup {
    
    public ScoreCommand(ArmSubsystem armSubsystem, SwerveSubsystem drivebase) {
        addCommands(
            new AdjustArmCommand(armSubsystem, -ArmConstants.PLACE_ROTATION_AMOUNT).withTimeout(ArmTravelTime.SCORE),
            new DriveToDistance(drivebase, -0.75).withTimeout(1.0)
        );
    }
}
