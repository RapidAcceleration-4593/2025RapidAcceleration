package frc.robot.commands.manual;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        elevatorSubsystem.setHardManualControl(!elevatorSubsystem.isHardManualControlEnabled());
        armSubsystem.setHardManualControl(!elevatorSubsystem.isHardManualControlEnabled());
        SmartDashboard.putBoolean("ManualControl", elevatorSubsystem.isHardManualControlEnabled());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
