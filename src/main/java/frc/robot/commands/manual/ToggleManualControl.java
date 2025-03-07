package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleManualControl extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;

    public ToggleManualControl(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        boolean newElevatorToggleState = !elevatorSubsystem.isManualControlEnabled();
        boolean newArmToggleState = !armSubsystem.isManualControlEnabled();

        elevatorSubsystem.setManualControl(newElevatorToggleState);
        armSubsystem.setManualControl(newArmToggleState);

        if (newElevatorToggleState || newArmToggleState) {
            double currentPosition = elevatorSubsystem.getEncoderValue();
            elevatorSubsystem.resetSetpoint(currentPosition);
            armSubsystem.resetSetpoint(currentPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
