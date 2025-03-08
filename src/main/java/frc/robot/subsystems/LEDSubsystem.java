package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
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

    private void displayStatus() {

        if (serializerSubsystem.isCoralLoaded()) {
            blinkingLEDs.set(LEDConstants.GREEN_SOLID);
        }
        else if (drivebase.isAutoDriving()) {
            blinkingLEDs.set(LEDConstants.BLUE_SOLID);
        }
        else if (elevatorSubsystem.isManualControlEnabled() || armSubsystem.isManualControlEnabled()) {
            blinkingLEDs.set(LEDConstants.RED_SOLID);
        }
        else {
            blinkingLEDs.set(0);
        }
    }

    public Command displayStatusCommand () {
        return run(this::displayStatus);
    }
}
