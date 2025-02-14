package frc.robot.commands.serializer.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class RunBeltReversedCommand extends Command {
    
    private final SerializerSubsystem serializerSubsystem;

    public RunBeltReversedCommand(SerializerSubsystem subsystem) {
        this.serializerSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.runBeltMotor(true);
    }

    @Override
    public void end(boolean interrupted) {
        serializerSubsystem.stopBeltMotor();
    }
}
