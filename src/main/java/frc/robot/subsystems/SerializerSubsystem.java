package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SerializerConstants;
import frc.robot.commands.armivator.KahChunkCommand;

public class SerializerSubsystem extends SubsystemBase {

    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    
    private final SparkMax serializerMotor = SerializerConstants.serializerMotor;
    private final DigitalInput serializerSensor = SerializerConstants.serializerSensor;

    private final SparkMaxConfig config = new SparkMaxConfig();

    private boolean hasTriggered = false;

    /**
     * Constructor for the SerializerSubsystem class.
     * Configures the motor settings.
     */
    public SerializerSubsystem(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        config.idleMode(IdleMode.kBrake);
        serializerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Serializer State System ----- */

    public void controlSerializerState() {
        updateValues();

        if (isCoralLoaded()) {
            if (!hasTriggered) {
                new KahChunkCommand(elevatorSubsystem, armSubsystem).schedule();
                stopSerializer();
                hasTriggered = true;
            }
        } else {
            hasTriggered = false;
            runSerializer(false);
        }
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
     * @param inverted Whether the serializer should spin reversely.
     */
    public void runSerializer(boolean inverted) {
        serializerMotor.set(inverted ? SerializerConstants.CONTROL_SPEED : -SerializerConstants.CONTROL_SPEED);
    }

    /** Stops the serializer belt motor. */
    public void stopSerializer() {
        serializerMotor.stopMotor();
    }

    /** Updates values to SmartDashboard/ShuffleBoard. */
    private void updateValues() {
        SmartDashboard.putBoolean("S-Sensor", isCoralLoaded());
    }
}