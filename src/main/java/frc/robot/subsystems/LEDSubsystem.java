package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private final Spark blinkingLEDs = LEDConstants.blinkingLEDs;

    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private final SerializerSubsystem serializerSubsystem;
    private final SwerveSubsystem drivebase;

    /**
     * Constructor for the LEDSubsystem class.
     * Configures the LED settings.
     */
    public LEDSubsystem(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SerializerSubsystem serializerSubsystem, SwerveSubsystem drivebase) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.serializerSubsystem = serializerSubsystem;
        this.drivebase = drivebase;
    }

    // I know using periodic is a terrible idea, but so is having direct references to subsystems. We are in the spagetti zone.
    @Override
    public void periodic() {
        // The order of the conditions is the order of priority
        // If hard manual control
        // blinkingLEDs.set(LEDConstants.RED_SOLID)
        // else if (serializer load)
        // blink green
        // else if autodrive
        // blink blue
        // else
        // turn off

    }
}
