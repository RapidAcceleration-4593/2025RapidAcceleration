package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.arm.AdjustArmCommand;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmivatorCommands {
    
    /** ElevatorSubsystem Object. */
    private final ElevatorSubsystem elevator;

    /** ArmSubsystem Object. */
    private final ArmSubsystem arm;

    /** Constructor for Armivator Commands. */
    public ArmivatorCommands(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevator = elevatorSubsystem;
        this.arm = armSubsystem;
    }

    /**
     * Command to set elevator and arm states while running PID Control.
     * @param elevatorState The selected state of the elevator.
     * @param armState The selected state of the arm.
     * @return A functional command to set the state of the elevator and arm. 
     */
    public Command setArmivatorState(ElevatorStates elevatorState, ArmStates armState) {
        return new SetArmivatorState(elevator, arm, elevatorState, armState);
    }

    /**
     * Command to set elevator states while running PID Control.
     * @param state The selected state of the elevator.
     * @return A functional command to set the state of the elevator.
     */
    public Command setElevatorState(ElevatorStates state) {
        return new SetElevatorState(elevator, state);
    }

    /**
     * Command to set arm states while running PID Control.
     * @param state The selected state of the arm.
     * @return A functional command to set the state of the arm.
     */
    public Command setArmState(ArmStates state) {
        return new SetArmState(arm, state);
    }

    /**
     * Command to adjust the arm setpoint while running PID Control.
     * @param adjustment The amount of encoder ticks to adjust the setpoint by.
     * @return A functional command to adjust the arm setpoint.
     */
    public Command adjustArmSetpoint(double adjustment) {
        return new AdjustArmCommand(arm, adjustment);
    }

    /**
     * Command to lower the setpoint of the arm while scoring.
     * @return A functional command to lower the arm setpoint.
     */
    public Command scoreCoral() {
        return new AdjustArmCommand(arm, -ArmConstants.PLACE_ROTATION_AMOUNT).withTimeout(ArmTravelTime.SCORE);
    }

    /**
     * A boolean of whether manual control is enabled.
     * @return Whether manual control is enabled.
     */
    public boolean isManualControlEnabled() {
        return elevator.isManualControlEnabled();
    }
}
