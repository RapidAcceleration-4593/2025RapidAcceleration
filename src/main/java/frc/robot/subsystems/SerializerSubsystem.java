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
    
    private final SparkMax motor = SerializerConstants.serializerMotor;
    private final DigitalInput sensor = SerializerConstants.serializerSensor;

    private final SparkMaxConfig config = new SparkMaxConfig();

    public SerializerSubsystem() {
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor speeds for the serializer.
     * @param speed The speed of the serializer motor.
     */
    public void setMotorSpeeds(boolean reversed) {
        motor.set(reversed ? SerializerConstants.CONTROL_SPEED : -SerializerConstants.CONTROL_SPEED);
    }

    /**
     * Stops the intake deploy motors.
     */
    public void stopMotors() {
        motor.stopMotor();
    }

    /**
     * Distance sensor to detect if coral is present.
     * @return Whether the coral is detected.
     */
    public boolean isCoralDetected() {
        return !sensor.get();
    }
}