package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleArmivatorManualControl extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;

    public ToggleArmivatorManualControl(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setManualControl(!elevatorSubsystem.isManualControlEnabled());
        armSubsystem.setManualControl(!armSubsystem.isManualControlEnabled());

        elevatorSubsystem.setSetpoint(elevatorSubsystem.getEncoderValue());
        armSubsystem.setSetpoint(armSubsystem.getEncoderValue());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
