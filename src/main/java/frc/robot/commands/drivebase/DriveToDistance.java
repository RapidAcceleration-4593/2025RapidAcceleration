package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToDistance extends SequentialCommandGroup {
    
    public DriveToDistance(SwerveSubsystem drivebase, double distance) {
        Pose2d currentPose = drivebase.getPose();
        Rotation2d currentRotation = currentPose.getRotation();

        addCommands(
            new DriveToPose(
                drivebase,
                new Pose2d(
                    currentPose.getTranslation().plus(new Translation2d(distance, 0).rotateBy(currentRotation)),
                    currentRotation
                ),
                2.5, 2.5
            )
        );
    }
}
