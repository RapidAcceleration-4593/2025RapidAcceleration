package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SerializerConstants;

public class SerializerSubsystem extends SubsystemBase {
    
    private final SparkMax serializerMotor = SerializerConstants.serializerMotor;

    private SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the SerializerSubsystem class.
     * Configures the motor settings.
     */
    public SerializerSubsystem() {
        config.idleMode(IdleMode.kBrake);

        serializerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Command Factory Methods ----- */

    /**
     * Runs the serializer belt, either forward or backward.
     * @param inverted Whether the serializer should spin reversely.
     */
    public void runSerializer(boolean inverted) {
        serializerMotor.set(inverted ? SerializerConstants.CONTROL_SPEED : -SerializerConstants.CONTROL_SPEED);
    }

    /** Stops the serializer belt motor. */
    public void stopSerializer() {
        serializerMotor.stopMotor();
    }
}