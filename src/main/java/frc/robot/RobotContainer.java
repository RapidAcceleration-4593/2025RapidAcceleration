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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.Constants.RobotStates.Arm.ArmDirections;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Autonomous.StartingPosition;
import frc.robot.Constants.RobotStates.Elevator.ElevatorDirections;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.arm.ControlArmState;
import frc.robot.commands.arm.AdjustArmCommand;
import frc.robot.commands.armivator.SetArmivatorState;
import frc.robot.commands.armivator.KahChunkCommand;
import frc.robot.commands.armivator.RemoveAlgaeCommand;
import frc.robot.commands.auton.MoveOutAuton;
import frc.robot.commands.auton.NoneAuton;
import frc.robot.commands.auton.OneCoralAuton;
import frc.robot.commands.auton.ThreeCoralAuton;
import frc.robot.commands.auton.TwoCoralAuton;
import frc.robot.commands.auton.utils.AutonUtils;
import frc.robot.commands.climber.RunClimberCommand;
import frc.robot.commands.elevator.ControlElevatorState;
import frc.robot.commands.manual.ManualArmCommand;
import frc.robot.commands.manual.ManualElevatorCommand;
import frc.robot.commands.manual.ToggleManualControl;
import frc.robot.commands.serializer.PickupCoralCommand;
import frc.robot.commands.serializer.RunSerializerCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final SerializerSubsystem serializerSubsystem = new SerializerSubsystem();
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Util(s)
    public final AutonUtils autonUtils = new AutonUtils(drivebase, elevatorSubsystem, armSubsystem, serializerSubsystem);
    public final PoseNavigator poseNavigator = new PoseNavigator(drivebase, autonUtils);

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController auxiliaryController = new CommandXboxController(OperatorConstants.AUXILIARY_CONTROLLER_PORT);

    /** DriveToPoseCommand for Acceleration Station Dashboard. */
    private Command driveToPoseCommand = Commands.none();

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
    }

    private void configureBindings() {
        driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

        // Autonomous Drive Control.
        driverController.leftTrigger()
            .whileTrue(Commands.runOnce(() -> {
                driveToPoseCommand = drivebase.driveToPose(
                    poseNavigator.selectTargetPose()
                );
                driveToPoseCommand.schedule();
            }))
            .onFalse(Commands.runOnce(() -> {
                if (driveToPoseCommand != Commands.none()) {
                    driveToPoseCommand.cancel();
                }
            }));

        // Armivator Control.
        driverController.rightTrigger().onTrue(new AdjustArmCommand(armSubsystem, -ArmConstants.PLACE_ROTATION_AMOUNT).withTimeout(ArmTravelTime.SCORE));

        driverController.leftBumper().onTrue(new PickupCoralCommand(elevatorSubsystem, armSubsystem, serializerSubsystem));
        driverController.rightBumper().onTrue(handleDashboardState());

        driverController.x().onTrue(new RemoveAlgaeCommand(elevatorSubsystem, armSubsystem, drivebase, poseNavigator));

        auxiliaryController.povUp().onTrue(new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.TOP, ArmStates.TOP));
        auxiliaryController.povRight().onTrue(new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.TOP));
        auxiliaryController.povLeft().onTrue(new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.L2));
        auxiliaryController.povDown().onTrue(new KahChunkCommand(elevatorSubsystem, armSubsystem));

        // Serializer Control.
        auxiliaryController.rightBumper().whileTrue(new RunSerializerCommand(serializerSubsystem, false));
        auxiliaryController.rightTrigger().whileTrue(new RunSerializerCommand(serializerSubsystem, true));

        // Climber Control.
        driverController.povUp().whileTrue(new RunClimberCommand(climberSubsystem, false));
        driverController.povDown().whileTrue(new RunClimberCommand(climberSubsystem, true));

        // Manual Control.
        auxiliaryController.back().onTrue(new ToggleManualControl(elevatorSubsystem, armSubsystem));

        auxiliaryController.y().whileTrue(new ManualElevatorCommand(elevatorSubsystem, ElevatorDirections.UP));
        auxiliaryController.a().whileTrue(new ManualElevatorCommand(elevatorSubsystem, ElevatorDirections.DOWN));

        auxiliaryController.x().whileTrue(new ManualArmCommand(armSubsystem, ArmDirections.UP));
        auxiliaryController.b().whileTrue(new ManualArmCommand(armSubsystem, ArmDirections.DOWN));
    }

    /** 
     * Handles the state of the armivator based on the value from the dashboard.
     * @return The command to run based on the dashboard selected state.
     */
    private Command handleDashboardState() {
        Map<Integer, Command> commandMap = Map.of(
            1, new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.BOTTOM),
            2, new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.L2),
            3, new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.TOP),
            4, new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.TOP, ArmStates.TOP)
        );

        return Commands.select(commandMap, () ->
            (int) SmartDashboard.getNumber("TargetArmivatorState", 1)
        );
    }

    /**
     * Returns the autonomous command to run. This will be run in autonomous mode.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        Map<String, Command> autonMap = Map.ofEntries(
            Map.entry("Do Nothing", new NoneAuton()),

            Map.entry("Left, Move Out", new MoveOutAuton(autonUtils, StartingPosition.LEFT)),
            Map.entry("Left, 1-Coral", new OneCoralAuton(autonUtils, StartingPosition.LEFT)),
            Map.entry("Left, 2-Coral", new TwoCoralAuton(autonUtils, StartingPosition.LEFT)),
            Map.entry("Left, 3-Coral", new ThreeCoralAuton(autonUtils, StartingPosition.LEFT)),

            Map.entry("Center, Move Out", new MoveOutAuton(autonUtils, StartingPosition.CENTER)),
            Map.entry("Center, 1-Coral", new OneCoralAuton(autonUtils, StartingPosition.CENTER)),

            Map.entry("Right, Move Out", new MoveOutAuton(autonUtils, StartingPosition.RIGHT)),
            Map.entry("Right, 1-Coral", new OneCoralAuton(autonUtils, StartingPosition.RIGHT)),
            Map.entry("Right, 2-Coral", new TwoCoralAuton(autonUtils, StartingPosition.RIGHT)),
            Map.entry("Right, 3-Coral", new ThreeCoralAuton(autonUtils, StartingPosition.RIGHT))
        );

        return Commands.select(autonMap, () -> 
            SmartDashboard.getString("SelectedAutonomous", "Do Nothing")
        );
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
