package frc.robot.commands.serializer.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class RunBeltCommand extends Command {
    
    private final SerializerSubsystem serializerSubsystem;

    public RunBeltCommand(SerializerSubsystem subsystem) {
        this.serializerSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.runBeltMotor();
    }

    @Override
    public void end(boolean interrupted) {
        serializerSubsystem.stopBeltMotor();
    }
}
