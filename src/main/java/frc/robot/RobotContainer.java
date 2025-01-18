// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auton.ExampleAuton;
import frc.robot.commands.drivebase.FieldCentricDrive;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {
    // Subsystem(s)
    public final static SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(0);

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
                                                                  () -> -driverController.getLeftY(),
                                                                  () -> -driverController.getLeftX())
                                                                .withControllerRotationAxis(() -> -driverController.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(1.0)
                                                                .allianceRelativeControl(false);


    /** Clones the angular velocity input stream and converts it to a fieldRelative input stream. */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                               driverController::getRightY)
                                                                                            .headingWhile(true);

    /** Clones the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                      .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                     () -> -driverController.getLeftY(),
                                                                     () -> -driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverController.getRawAxis(2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

    /** Derive the heading axis with math. */
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(() -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                   () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
        .headingWhile(true);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        // Command driveRobotOrientedAngularVelocity         = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedAngularVelocity         = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        // Command driveFieldOrientedDirectAngle             = drivebase.driveFieldOriented(driveDirectAngle);
        // Command driveFieldOrientedDirectAngleKeyboard     = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        // Command driveSetpointGen                          = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle); 
        // Command driveSetpointGenKeyboard                  = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

        // (Condition) ? Return-On-True : Return-On-False.
        drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                    driveFieldOrientedAngularVelocity :
                                    driveFieldOrientedAngularVelocityKeyboard);

        driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

        // Dashboard input for driving to branch pose based on alliance side.
        new Trigger(() -> SmartDashboard.getBoolean("ConfirmedCondition", false))
            .onTrue(Commands.runOnce(() -> {
                drivebase.driveToPose(drivebase.findBranchPose(
                        0.5,
                        drivebase.targetReefBranch,
                        drivebase.isRedAlliance()
                    )).schedule();
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
