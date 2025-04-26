package frc.robot.commands.drivebase;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.PoseNavigator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToClosestReef extends DeferredCommand {
    
    public DriveToClosestReef(SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        super(
            () -> drivebase.driveToPose(poseNavigator.calculateClosestReefPose()),
            Set.of(drivebase)
        );
    }
}
