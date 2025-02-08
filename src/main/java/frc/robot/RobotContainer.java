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
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.arm.manual.MoveArmDown;
import frc.robot.commands.arm.manual.MoveArmUp;
import frc.robot.commands.auton.NoneAuton;
import frc.robot.commands.auton.utils.AutonUtils;
import frc.robot.commands.drivebase.FieldCentricDrive;
import frc.robot.commands.elevator.MaintainElevatorLevel;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.elevator.manual.MoveElevatorDown;
import frc.robot.commands.elevator.manual.MoveElevatorUp;
import frc.robot.commands.intake.ManageIntake;
import frc.robot.commands.intake.left.RunLeftIntake;
import frc.robot.commands.intake.left.RunLeftIntakeReverse;
import frc.robot.commands.intake.right.RunRightIntake;
import frc.robot.commands.intake.right.RunRightIntakeReverse;
import frc.robot.commands.serializer.ControlSerializerBelt;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseNavigator;
import frc.robot.subsystems.SerializerSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {
    // Subsystem(s)
    public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem(elevatorSubsystem);
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final SerializerSubsystem serializerSubsystem = new SerializerSubsystem();

    // Util(s)
    public final AutonUtils autonUtils = new AutonUtils(drivebase);
    public final PoseNavigator poseNavigator = new PoseNavigator(autonUtils);

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController auxiliaryControler = new CommandXboxController(OperatorConstants.AUXILIARY_CONTROLLER_PORT);

    /** DriveToPoseCommand for Acceleration Station Dashboard. */
    private Command driveToPoseCommand = null;

    /** Swerve Drive Command with full field-centric mode and heading correction. */
    FieldCentricDrive fieldCentricDrive = new FieldCentricDrive(drivebase,
                                                                () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                                OperatorConstants.DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                                OperatorConstants.DEADBAND),
                                                                () -> MathUtil.applyDeadband(driverController.getRightX(),
                                                                                                OperatorConstants.DEADBAND),
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
                                                                .scaleTranslation(OperatorConstants.SCALE_TRANSLATION)
                                                                .allianceRelativeControl(true);

    /** Clones the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                      .allianceRelativeControl(false);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        elevatorSubsystem.setDefaultCommand(new MaintainElevatorLevel(elevatorSubsystem));
        // armSubsystem.setDefaultCommand(new MaintainArmAngle(armSubsystem));
        intakeSubsystem.setDefaultCommand(new ManageIntake(intakeSubsystem));
        serializerSubsystem.setDefaultCommand(new ControlSerializerBelt(serializerSubsystem));
    }

    private void configureBindings() {
        // (Condition) ? Return-On-True : Return-On-False.
        driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

        driverController.leftTrigger()
            .whileTrue(Commands.runOnce(() -> {
                driveToPoseCommand = drivebase.driveToPose(
                    poseNavigator.selectTargetPose(AutonConstants.DISTANCE_FROM_REEF, drivebase.isRedAlliance())
                );
                driveToPoseCommand.schedule();
            }))
            .onFalse(Commands.runOnce(() -> {
                if (driveToPoseCommand != null) {
                    driveToPoseCommand.cancel();
                }
            }));

        // Manual Elevator Control for Testing Purposes.
        driverController.y().whileTrue(new MoveElevatorUp(elevatorSubsystem));
        driverController.a().whileTrue(new MoveElevatorDown(elevatorSubsystem));

        // Manual Arm Control for Testing Purposes.
        driverController.x().whileTrue(new MoveArmUp(armSubsystem));
        driverController.b().whileTrue(new MoveArmDown(armSubsystem));

        // Robot Functionality
        driverController.rightBumper().onTrue(Commands.runOnce(armSubsystem::placeCoralCommand));
        driverController.leftBumper().onTrue(Commands.runOnce(armSubsystem::homeArmivatorCommand));

        // Elevator PID Setpoints.
        driverController.povUp().onTrue(new SetElevatorSetpoint(elevatorSubsystem, ElevatorStates.L4));
        driverController.povLeft().onTrue(new SetElevatorSetpoint(elevatorSubsystem, ElevatorStates.L3));
        driverController.povRight().onTrue(new SetElevatorSetpoint(elevatorSubsystem, ElevatorStates.PICKUP));
        driverController.povDown().onTrue(new SetElevatorSetpoint(elevatorSubsystem, ElevatorStates.BOTTOM));

        // Intake Commands
        auxiliaryControler.leftBumper().onTrue(Commands.runOnce(intakeSubsystem::toggleLeftIntakeCommand));
        auxiliaryControler.rightBumper().onTrue(Commands.runOnce(intakeSubsystem::toggleRightIntakeCommand));

        auxiliaryControler.x().whileTrue(new RunLeftIntake(intakeSubsystem));
        auxiliaryControler.a().whileTrue(new RunLeftIntakeReverse(intakeSubsystem));

        auxiliaryControler.b().whileTrue(new RunRightIntake(intakeSubsystem));
        auxiliaryControler.y().whileTrue(new RunRightIntakeReverse(intakeSubsystem));
    }

    /**
     * Returns the autonomous command to run. This will be run in autonomous mode.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        // TODO: Implement autonomous selection for dashboard.
        return new NoneAuton();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
