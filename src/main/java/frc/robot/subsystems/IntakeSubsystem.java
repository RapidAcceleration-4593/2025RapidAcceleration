package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leftIntakeMotor = IntakeConstants.leftIntakeMotor;
    private final PWMSparkMax rightIntakeMotor = IntakeConstants.rightIntakeMotor;
    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the IntakeSubsystem class.
     * Configures the motor settings and sets the idle mode to brake.
     */
    public IntakeSubsystem() {
        config.idleMode(IdleMode.kBrake);
        leftIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    /** ----- Factory Command Methods ----- */

    /**
     * Runs the intake motors at a pre-defined speed.
     * @param direction The direction to move the intake.
     */
    public void runIntake(boolean reversed) {
        leftIntakeMotor.set(reversed ? IntakeConstants.CONTROL_SPEED : -IntakeConstants.CONTROL_SPEED);
        rightIntakeMotor.set(reversed ? 0.8 : -0.8);
    }

    /**
     * Runs the intake motors to hold the coral for L1 scoring.
     */
    public void storeCoral() {
        leftIntakeMotor.set(-IntakeConstants.CONTROL_SPEED);
        rightIntakeMotor.set(IntakeConstants.CONTROL_SPEED);
    }

    /** Stops the intake motors, putting them in brake mode. */
    public void stopIntake() {
        leftIntakeMotor.stopMotor();
        rightIntakeMotor.stopMotor();
    }
}
