package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor = ClimberConstants.climberMotor;
    private final SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the ClimberSubsystem class.
     * Configures the motor settings and sets the idle mode to brake.
     */
    public ClimberSubsystem() {
        config.idleMode(IdleMode.kBrake);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    /** ----- Factory Command Methods ----- */

    /**
     * Runs the climber motor at a pre-defined speed.
     * @param direction The direction to move the climber.
     */
    public void runClimber(boolean reversed) {
        climberMotor.set(reversed ? -ClimberConstants.CONTROL_SPEED : ClimberConstants.CONTROL_SPEED);
    }

    /** Stops the climber motor, putting it in brake mode. */
    public void stopClimber() {
        climberMotor.stopMotor();
    }
}
