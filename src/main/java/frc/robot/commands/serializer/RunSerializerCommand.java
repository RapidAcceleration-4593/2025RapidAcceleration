package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class RunSerializerCommand extends Command {
    
    private final SerializerSubsystem serializerSubsystem;
    private final boolean isInverted;
    private final boolean isTerminable;
    
    public RunSerializerCommand(SerializerSubsystem subsystem, boolean isInverted, boolean isTerminable) {
        this.serializerSubsystem = subsystem;
        this.isInverted = isInverted;
        this.isTerminable = isTerminable;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        serializerSubsystem.setMotorSpeeds(isInverted);
    }

    @Override
    public void end(boolean interrupted) {
        serializerSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return isTerminable && serializerSubsystem.isCoralDetected();
    }
}
