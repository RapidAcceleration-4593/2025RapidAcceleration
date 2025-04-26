package frc.robot.commands.drivebase;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToDistance extends DeferredCommand {
    
    public DriveToDistance(SwerveSubsystem drivebase, double distance) {
        super(
            () -> {
                Pose2d currentPose = drivebase.getPose();
                Rotation2d currentRotation = currentPose.getRotation();
                
                Pose2d targetPose = new Pose2d(
                    currentPose.getTranslation().plus(new Translation2d(distance, 0).rotateBy(currentRotation)),
                    currentRotation
                );
                
                return drivebase.driveToPose(targetPose, 2.5, 2.5);
            },
            Set.of(drivebase)
        );
    }
}
