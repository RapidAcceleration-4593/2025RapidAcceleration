package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class RunSerializerCommand extends Command {
    
    private final SerializerSubsystem serializerSubsystem;
    private final boolean serializerInverted;

    public RunSerializerCommand(SerializerSubsystem subsystem, boolean inverted) {
        this.serializerSubsystem = subsystem;
        this.serializerInverted = inverted;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.runSerializer(serializerInverted);
    }

    @Override
    public void end(boolean interrupted) {
        serializerSubsystem.stopSerializer();
    }
}
