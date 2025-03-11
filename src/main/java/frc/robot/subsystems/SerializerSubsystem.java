package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SerializerConstants;

public class SerializerSubsystem extends SubsystemBase {
    
    private final SparkMax serializerMotor = SerializerConstants.serializerMotor;
    private final DigitalInput serializerSensor = SerializerConstants.serializerSensor;

    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the SerializerSubsystem class.
     * Configures the motor settings.
     */
    public SerializerSubsystem() {
        config.idleMode(IdleMode.kBrake);
        serializerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Sensor Abstraction ----- */

    /**
     * Is there a coral ready in the serializer?
     * @return If the coral sensor is triggered.
     */
    public boolean isCoralLoaded() {
        return !serializerSensor.get();
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Runs the serializer belt, either forward or backward.
     * @param reversed Whether the serializer should spin reversely.
     */
    public void runSerializer(boolean reversed) {
        serializerMotor.set(reversed ? SerializerConstants.CONTROL_SPEED : -SerializerConstants.CONTROL_SPEED);
    }

    /** Stops the serializer belt motor. */
    public void stopSerializer() {
        serializerMotor.stopMotor();
    }

    /**
     * Runs the serializer belt motor until the coral sensor is triggered.
     * @return A command that runs the serializer.
     */
    public Command runSerializerCommand() {
        return run(() -> runSerializer(false))
            .finallyDo(this::stopSerializer)
            .until(this::isCoralLoaded);
    }
}