package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class RunSerializerCommand extends Command {
    
    private final SerializerSubsystem serializerSubsystem;
    private final boolean isInverted;
    
    public RunSerializerCommand(SerializerSubsystem subsystem, boolean isInverted) {
        this.serializerSubsystem = subsystem;
        this.isInverted = isInverted;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.runSerializer(isInverted);
    }

    @Override
    public void end(boolean interrupted) {
        serializerSubsystem.stopSerializer();
    }
}
