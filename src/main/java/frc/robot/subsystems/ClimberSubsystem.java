package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotStates.Climber.ClimberDirections;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax leftClimberMotor = ClimberConstants.leftClimberMotor;
    private final SparkMax rightClimberMotor = ClimberConstants.rightClimberMotor;

    private final SparkMaxConfig brakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig coastConfig = new SparkMaxConfig();

    /**
     * Constructor for the ClimberSubsystem class.
     * Configures the motor settings and sets the idle mode to brake.
     */
    public ClimberSubsystem() {
        brakeConfig.idleMode(IdleMode.kBrake);
        coastConfig.idleMode(IdleMode.kCoast);

        leftClimberMotor.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightClimberMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    /** ----- Factory Command Methods ----- */

    /**
     * Runs the climber motor at a pre-defined speed.
     * @param direction The direction to move the climber.
     */
    public void runClimber(ClimberDirections direction) {
        switch (direction) {
            case IN:
                rightClimberMotor.set(-ClimberConstants.CONTROL_SPEED); // NEO V1.1
                break;
            case OUT:
                rightClimberMotor.set(ClimberConstants.CONTROL_SPEED);  // NEO 550
                break;
            default:
                break;
        }
    }

    /** Stops the climber motor, putting it in brake mode. */
    public void stopClimber() {
        leftClimberMotor.stopMotor();
        rightClimberMotor.stopMotor();
    }
}
