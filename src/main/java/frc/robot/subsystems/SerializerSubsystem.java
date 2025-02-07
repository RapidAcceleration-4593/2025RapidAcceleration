package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SerializerConstants;

public class SerializerSubsystem extends SubsystemBase {
    
    private final SparkMax beltMotor = SerializerConstants.beltMotor;

    private final DigitalInput beltLimitSwitch = SerializerConstants.beltLimitSwitch;

    private SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the SerializerSubsystem class.
     * Configures the motor settings.
     */
    public SerializerSubsystem() {
        config.idleMode(IdleMode.kBrake);

        beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Belt State Management ----- */

    /**
     * Checks if the belt limit switch is pressed when a coral is detected.
     * Stops the motor when a coral is detected; run otherwise.
     */
    public void controlBeltState() {
        if (isCoralDetected()) {
            stopBeltMotor();
        } else {
            runBeltMotor(0.25); // TODO: Adjust speed if needed.
        }
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Checks if the coral is detected by the limit switch.
     * @return Whether {@link SerializerSubsystem#beltLimitSwitch} is pressed.
     */
    private boolean isCoralDetected() {
        return !beltLimitSwitch.get();
    }


    /** ----- Motor Control ----- */

    /** Runs the belt motor at a defined speed. */
    private void runBeltMotor(double speed) {
        beltMotor.set(speed); // TODO: Invert if needed.
    }

    /** Stops the belt motor; sets speed to zero. */
    private void stopBeltMotor() {
        beltMotor.stopMotor();
    }
}