package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class ControlSerializerState extends Command {
    
    private final SerializerSubsystem serializerSubsystem;

    public ControlSerializerState(SerializerSubsystem subsystem) {
        this.serializerSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.controlSerializerState();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
