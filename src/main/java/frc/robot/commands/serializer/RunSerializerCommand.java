package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class RunSerializerCommand extends Command {
    
    private final SerializerSubsystem serializerSubsystem;
    private final boolean serializerReversed;

    public RunSerializerCommand(SerializerSubsystem subsystem, boolean reversed) {
        this.serializerSubsystem = subsystem;
        this.serializerReversed = reversed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.runSerializer(serializerReversed);
    }

    @Override
    public void end(boolean interrupted) {
        serializerSubsystem.stopSerializer();
    }
}
