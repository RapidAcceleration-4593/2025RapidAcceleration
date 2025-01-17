package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    /** Creates a new Example Subsystem. */
    public ExampleSubsystem() {
        // Initialize the Example Subsystem here
    }

    /**
     * Example command factory method.
     * @return A command to be scheduled.
     */
    public Command exampleMethodCommand() {
        // Inline construction of a command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                /* One-time action goes here. */
            }
        );
    }

    /**
     * An example method querying a boolean state of the subsystem.
     * @return A value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state.
        return false;
    }
}
