package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private final Spark BlinkingLEDs = LEDConstants.BlinkingLEDs;

    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private final SerializerSubsystem serializerSubsystem;

    /**
     * Constructor for the LEDSubsystem class.
     * Configures the LED settings.
     */
    public LEDSubsystem(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SerializerSubsystem serializerSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.serializerSubsystem = serializerSubsystem;
    }
}
