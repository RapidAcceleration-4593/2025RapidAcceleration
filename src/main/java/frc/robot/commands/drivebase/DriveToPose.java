package frc.robot.commands.drivebase;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPose extends DeferredCommand {

    public DriveToPose(SwerveSubsystem drivebase, Pose2d targetPose) {
        this(drivebase, targetPose, AutonConstants.MAX_VELOCITY, AutonConstants.MAX_ACCELERATION);
    }

    public DriveToPose(SwerveSubsystem drivebase, Pose2d targetPose, double velocity, double acceleration) {
        super(() -> drivebase.driveToPose(targetPose, velocity, acceleration),
            Set.of(drivebase)
        );
    }
}
