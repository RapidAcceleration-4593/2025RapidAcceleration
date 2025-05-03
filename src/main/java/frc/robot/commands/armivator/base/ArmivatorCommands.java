package frc.robot.commands.armivator.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.arm.AdjustArmCommand;
import frc.robot.commands.arm.base.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.serializer.RunSerializerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SerializerSubsystem;

public class ArmivatorCommands {
    
    /** ElevatorSubsystem Object. */
    private final ElevatorSubsystem elevator;

    /** ArmSubsystem Object. */
    private final ArmSubsystem arm;

    /** SerializerSubsystem Object. */
    private final SerializerSubsystem serializer;

    /** Constructor for Armivator Commands. */
    public ArmivatorCommands(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SerializerSubsystem serializerSubsystem) {
        this.elevator = elevatorSubsystem;
        this.arm = armSubsystem;
        this.serializer = serializerSubsystem;
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
     * Command to lower the setpoint of the arm while scoring.
     * @return A functional command to lower the arm setpoint.
     */
    public Command scoreCoral() {
        return new AdjustArmCommand(arm, -ArmConstants.PLACE_ROTATION_AMOUNT).withTimeout(ArmTravelTime.SCORE);
    }

    /**
     * Command to run the serializer until the sensor is triggered.
     * @return A functional command to run the serializer.
     */
    public Command runSerializerCommand() {
        return new RunSerializerCommand(serializer, false, true);
    }

    /**
     * A boolean of whether manual control is enabled.
     * @return Whether manual control is enabled.
     */
    public boolean isManualControlEnabled() {
        return elevator.isManualControlEnabled();
    }
}
