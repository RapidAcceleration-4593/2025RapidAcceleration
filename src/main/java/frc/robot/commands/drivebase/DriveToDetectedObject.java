package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;
import java.util.Set;

public class DriveToDetectedObject extends DeferredCommand {

    public DriveToDetectedObject(SwerveSubsystem drivebase) {
        super(
            () -> {
                Optional<Pose2d> finalPose = drivebase.visionUtils.getDetectedObjectPose(drivebase.getPose());
                return finalPose.map(drivebase::driveToPose).orElse(Commands.none());
            },
            Set.of(drivebase)
        );
    }
}
