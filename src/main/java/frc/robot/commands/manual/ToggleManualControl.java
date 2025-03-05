package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleManualControl extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private boolean doManualControl;

    public ToggleManualControl(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, boolean doManualControl) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.doManualControl = doManualControl;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.toggleManualControl(doManualControl);
        armSubsystem.toggleManualControl(doManualControl);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
