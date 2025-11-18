// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands
     */
    public RobotContainer() {
        configureBindings();
    }

    /**
     * Configures controller bindings
     */
    private void configureBindings() {
        driverController.a().onTrue(new SetElevatorState(elevatorSubsystem, ElevatorStates.TOP));
    }

    /**
     * Returns the autonomous command to run. This will be ran in autonomous mode.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void setMotorBrake(boolean brake) {

    }
}
