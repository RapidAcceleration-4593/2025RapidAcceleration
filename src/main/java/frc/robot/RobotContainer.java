// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStates.ControlDirections;
import frc.robot.Constants.RobotStates.StartingPosition;
import frc.robot.commands.arm.ScoreCommand;
import frc.robot.commands.arm.base.ControlArmState;
import frc.robot.commands.armivator.HandleDashboardState;
import frc.robot.commands.armivator.PickupCoralCommand;
import frc.robot.commands.armivator.RemoveAlgaeCommand;
import frc.robot.commands.armivator.base.ArmivatorCommands;
import frc.robot.commands.auton.MoveOutAuton;
import frc.robot.commands.auton.NoneAuton;
import frc.robot.commands.auton.OneCoralAuton;
import frc.robot.commands.auton.TwoHalfCoralAuton;
import frc.robot.commands.auton.TwoCoralAuton;
import frc.robot.commands.auton.utils.AutonUtils;
import frc.robot.commands.drivebase.DriveToDashboardPose;
import frc.robot.commands.drivebase.DriveToDetectedObject;
import frc.robot.commands.elevator.ControlElevatorState;
import frc.robot.commands.intake.IntakeL1Command;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.base.ControlIntakeState;
import frc.robot.commands.manual.ManualArmCommand;
import frc.robot.commands.manual.ManualElevatorCommand;
import frc.robot.commands.manual.ToggleArmivatorManualControl;
import frc.robot.commands.manual.ToggleIntakeManualControl;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.utils.PoseNavigator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {

    // Subsystem(s)
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final SerializerSubsystem serializerSubsystem = new SerializerSubsystem();
    private final IntakeDeploySubsystem intakeDeploySubsystem = new IntakeDeploySubsystem();
    private final IntakeFeederSubsystem intakeFeederSubsystem = new IntakeFeederSubsystem();

    // Util(s)
    private final ArmivatorCommands armivatorCommands = new ArmivatorCommands(elevatorSubsystem, armSubsystem, serializerSubsystem);
    private final AutonUtils autonUtils = new AutonUtils(drivebase);
    private final PoseNavigator poseNavigator = new PoseNavigator(drivebase, armivatorCommands);

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController auxiliaryController = new CommandXboxController(OperatorConstants.AUXILIARY_CONTROLLER_PORT);

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
        elevatorSubsystem.setDefaultCommand(new ControlElevatorState(elevatorSubsystem));
        armSubsystem.setDefaultCommand(new ControlArmState(armSubsystem));
        intakeDeploySubsystem.setDefaultCommand(new ControlIntakeState(intakeDeploySubsystem));
    }

    private void configureBindings() {
        /* --- Drivebase Control --- */
        driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

        /* --- Autonomous Navigation --- */
        driverController.leftTrigger().whileTrue(new DriveToDashboardPose(drivebase, poseNavigator));
        driverController.povDown().whileTrue(new DriveToDetectedObject(drivebase));
        driverController.x().onTrue(new RemoveAlgaeCommand(drivebase, armivatorCommands, poseNavigator));

        /* --- Armivator Control --- */
        driverController.rightTrigger().onTrue(new ScoreCommand(armSubsystem, drivebase));

        driverController.leftBumper().onTrue(new PickupCoralCommand(armivatorCommands, serializerSubsystem, intakeDeploySubsystem, intakeFeederSubsystem));
        driverController.rightBumper().onTrue(new HandleDashboardState(armivatorCommands, poseNavigator));

        /* --- Intake Control --- */
        driverController.a().whileTrue(new IntakeL1Command(intakeDeploySubsystem, intakeFeederSubsystem, armivatorCommands));
        driverController.a().whileTrue(new RunIntakeCommand(intakeFeederSubsystem, true));

        /* --- Manual Control --- */
        auxiliaryController.start().onTrue(new ToggleIntakeManualControl(intakeDeploySubsystem));
        auxiliaryController.back().onTrue(new ToggleArmivatorManualControl(elevatorSubsystem, armSubsystem));

        auxiliaryController.y().whileTrue(new ManualElevatorCommand(elevatorSubsystem, ControlDirections.UP));
        auxiliaryController.a().whileTrue(new ManualElevatorCommand(elevatorSubsystem, ControlDirections.DOWN));

        auxiliaryController.x().whileTrue(new ManualArmCommand(armSubsystem, ControlDirections.UP));
        auxiliaryController.b().whileTrue(new ManualArmCommand(armSubsystem, ControlDirections.DOWN));
    }

    /**
     * Returns the autonomous command to run. This will be ran in autonomous mode.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        Map<String, Command> autonMap = Map.ofEntries(
            Map.entry("Do Nothing", new NoneAuton()),

            Map.entry("Left, Move Out", new MoveOutAuton(autonUtils, StartingPosition.LEFT)),
            Map.entry("Left, 1-Coral", new OneCoralAuton(armivatorCommands, autonUtils, StartingPosition.LEFT)),
            Map.entry("Left, 2-Coral", new TwoCoralAuton(armivatorCommands, autonUtils, StartingPosition.LEFT)),
            Map.entry("Left, 2.5-Coral", new TwoHalfCoralAuton(armivatorCommands, autonUtils, StartingPosition.LEFT)),

            Map.entry("Center, Move Out", new MoveOutAuton(autonUtils, StartingPosition.CENTER)),
            Map.entry("Center, 1-Coral", new OneCoralAuton(armivatorCommands, autonUtils, StartingPosition.CENTER)),

            Map.entry("Right, Move Out", new MoveOutAuton(autonUtils, StartingPosition.RIGHT)),
            Map.entry("Right, 1-Coral", new OneCoralAuton(armivatorCommands, autonUtils, StartingPosition.RIGHT)),
            Map.entry("Right, 2-Coral", new TwoCoralAuton(armivatorCommands, autonUtils, StartingPosition.RIGHT)),
            Map.entry("Right, 2.5-Coral", new TwoHalfCoralAuton(armivatorCommands, autonUtils, StartingPosition.RIGHT))
        );

        return Commands.select(autonMap, () -> 
            SmartDashboard.getString("SelectedAutonomous", "Do Nothing")
        );
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
