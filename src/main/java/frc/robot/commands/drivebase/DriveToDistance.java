package frc.robot.commands.drivebase;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.drivebase.base.DriveToPose;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToDistance extends DeferredCommand {
    
    public DriveToDistance(SwerveSubsystem drivebase, double distance) {
        super(() -> 
            new DriveToPose(
                drivebase,
                new Pose2d(
                    drivebase.getPose().getTranslation().plus(new Translation2d(distance, 0).rotateBy(drivebase.getPose().getRotation())),
                    drivebase.getPose().getRotation()
                ),
                3.0, 3.0
            ), Set.of(drivebase)
        );
    }
}
