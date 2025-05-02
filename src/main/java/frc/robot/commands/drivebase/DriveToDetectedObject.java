package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToDetectedObject extends SequentialCommandGroup {

    public DriveToDetectedObject(SwerveSubsystem drivebase) {
        addCommands(
            new CloseDriveToPose(drivebase, () -> drivebase.getVisionUtils().getDetectedObjectPose(drivebase.getPose()))
        );
    }
}
