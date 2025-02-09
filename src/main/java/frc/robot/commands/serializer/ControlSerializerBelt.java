package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class ControlSerializerBelt extends Command {
    
    private final SerializerSubsystem serializerSubsystem;

    public ControlSerializerBelt(SerializerSubsystem subsystem) {
        this.serializerSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.controlBeltState();
    }
}
