package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax leaderClimberMotor = ClimberConstants.leftClimberMotor;
    private final SparkMax followerClimberMotor = ClimberConstants.rightClimberMotor;

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    /**
     * Constructor for the ClimberSubsystem class.
     * Configures the motor settings and sets the idle mode to brake.
     */
    public ClimberSubsystem() {
        leaderConfig.idleMode(IdleMode.kBrake);
        followerConfig.idleMode(IdleMode.kBrake).follow(leaderClimberMotor, true);

        leaderClimberMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerClimberMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    /** ----- Factory Command Methods ----- */

    /**
     * Runs the climber motor at a pre-defined speed.
     * @param inverted Whether the climber should spin reveresely.
     */
    public void runClimber(boolean inverted) {
        leaderClimberMotor.set(inverted ? ClimberConstants.CONTROL_SPEED : -ClimberConstants.CONTROL_SPEED);
    }

    /** Stops the climber motor, putting it in brake mode. */
    public void stopClimber() {
        leaderClimberMotor.stopMotor();
    }
}
