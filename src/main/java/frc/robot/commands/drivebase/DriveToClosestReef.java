package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.utils.PoseNavigator;

public class DriveToClosestReef extends SequentialCommandGroup {
    
    public DriveToClosestReef(SwerveSubsystem drivebase, PoseNavigator poseNavigator, double distance) {
        addCommands(
            new DriveToPose(drivebase, poseNavigator.calculateClosestReefPose(distance))
        );
    }
}
