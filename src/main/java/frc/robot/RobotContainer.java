// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auton.ExampleAuton;
import frc.robot.commands.drivebase.FieldCentricDrive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PoseNavigator;
import swervelib.SwerveInputStream;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {
    // Subsystem(s)
    public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Util(s)
    public final PoseNavigator poseNavigator = new PoseNavigator();

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(0);

    /** DriveToPoseCommand for Custom Dashboard. */
    private Command driveToPoseCommand = null;

    /** Swerve Drive Command with full field-centric mode and heading correction. */
    FieldCentricDrive fieldCentricDrive = new FieldCentricDrive(drivebase,
                                                                () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                                OperatorConstants.DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverController.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                driverController.getHID()::getYButtonPressed,
                                                                driverController.getHID()::getAButtonPressed,
                                                                driverController.getHID()::getXButtonPressed,
                                                                driverController.getHID()::getBButtonPressed);

    /** Converts driver input into a field-relative ChassisSpeeds that is controller by angular velocity. */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverController.getLeftY() * -1,
                                                                  () -> driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(() -> -driverController.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);

    /** Clones the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                      .allianceRelativeControl(false);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        // Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        // (Condition) ? Return-On-True : Return-On-False.
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

        driverController.leftTrigger()
            .whileTrue(Commands.runOnce(() -> {
                driveToPoseCommand = drivebase.driveToPose(
                    poseNavigator.selectTargetPose(0.7, drivebase.isRedAlliance())
                );
                driveToPoseCommand.schedule();
            }))
            .onFalse(Commands.runOnce(() -> {
                if (driveToPoseCommand != null) {
                    driveToPoseCommand.cancel();
                }
            }));
    }

    /**
     * Returns the autonomous command to run. This will be run in autonomous mode.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return new ExampleAuton();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
