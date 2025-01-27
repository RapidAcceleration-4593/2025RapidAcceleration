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
import frc.robot.Constants.SwingArmConstants.SwingArmState;
import frc.robot.commands.auton.NoneAuton;
import frc.robot.commands.drivebase.FieldCentricDrive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwingArmSubsystem;
import frc.robot.subsystems.PoseNavigator;
import swervelib.SwerveInputStream;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {
    // Subsystem(s)
    public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public static final SwingArmSubsystem arm = new SwingArmSubsystem();

    // Util(s)
    public final PoseNavigator poseNavigator = new PoseNavigator();

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(0);

    /** DriveToPoseCommand for Custom Dashboard. */
    private Command driveToPoseCommand = null;

    /** Swerve Drive Command with full field-centric mode and heading correction. */
    FieldCentricDrive fieldCentricDrive = new FieldCentricDrive(drivebase,
                                                                () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                                OperatorConstants.DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                                OperatorConstants.DEADBAND),
                                                                () -> MathUtil.applyDeadband(driverController.getRightX(),
                                                                                                OperatorConstants.DEADBAND),
                                                                driverController.getHID()::getAButtonPressed,
                                                                driverController.getHID()::getYButtonPressed,
                                                                driverController.getHID()::getBButtonPressed,
                                                                driverController.getHID()::getXButtonPressed);

    /** Converts driver input into a field-relative ChassisSpeeds that is controller by angular velocity. */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  driverController::getLeftY,
                                                                  driverController::getLeftX)
                                                                .withControllerRotationAxis(() -> -driverController.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(OperatorConstants.SCALE_TRANSLATION)
                                                                .allianceRelativeControl(true);

    /** Clones the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                      .allianceRelativeControl(false);

    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        drivebase.setDefaultCommand(fieldCentricDrive);
        arm.setDefaultCommand(arm.controlArmStatePIDCommand());
        
        driverController.y().whileTrue(arm.moveArmUpCommand());
        driverController.y().onFalse(arm.stopArmCommand());

        driverController.a().whileTrue(arm.moveArmDownCommand());
        driverController.a().onFalse(arm.stopArmCommand());

        driverController.povUp().onTrue(arm.moveToPositionCommand(SwingArmState.BOTTOM));
        driverController.povRight().onTrue(arm.moveToPositionCommand(SwingArmState.L3));
        driverController.povDown().onTrue(arm.moveToPositionCommand(SwingArmState.L4));
        driverController.povLeft().onTrue(arm.moveToPositionCommand(SwingArmState.TOP));
        
        driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

        driverController.leftTrigger()
            .whileTrue(Commands.runOnce(() -> {
                driveToPoseCommand = drivebase.driveToPose(
                    poseNavigator.selectTargetPose(1.0, drivebase.isRedAlliance())
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
        return new NoneAuton();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
