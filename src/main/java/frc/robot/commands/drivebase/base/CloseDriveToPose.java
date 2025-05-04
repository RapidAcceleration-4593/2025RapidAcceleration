package frc.robot.commands.drivebase.base;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class CloseDriveToPose extends Command {

    private final SwerveSubsystem drivebase;
    private final Supplier<Optional<Pose2d>> poseSupplier;

    private Pose2d lastKnownPose;

    private final PIDController xController = new PIDController(1.5, 0.0, 0.0);
    private final PIDController yController = new PIDController(1.5, 0.0, 0.0);
    private final PIDController rotationController = new PIDController(3.0, 0.0, 0.0);

    public CloseDriveToPose(SwerveSubsystem drivebase, Supplier<Optional<Pose2d>> poseSupplier) {
        this.drivebase = drivebase;
        this.poseSupplier = poseSupplier;

        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        rotationController.setTolerance(Units.degreesToRadians(1));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        lastKnownPose = poseSupplier.get().orElse(drivebase.getPose());

        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    @Override
    public void execute() {
        poseSupplier.get().ifPresent(pose -> lastKnownPose = pose);

        Pose2d currentPose = drivebase.getPose();
        
        xController.setSetpoint(lastKnownPose.getX());
        yController.setSetpoint(lastKnownPose.getY());
        rotationController.setSetpoint(lastKnownPose.getRotation().getRadians());

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotationSpeed = rotationController.calculate(currentPose.getRotation().getRadians());
    
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rotationSpeed,
            currentPose.getRotation()
        );

        drivebase.setChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    }
}
