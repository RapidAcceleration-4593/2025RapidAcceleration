package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private final SparkMax climberMotor = ClimberConstants.climberMotor;

    private final SparkMaxConfig config = new SparkMaxConfig();

    public ClimberSubsystem() {
        config.idleMode(IdleMode.kBrake);

        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Command Factory Methods ----- */
    
    public void runClimber(boolean reversed) {
        double speed = reversed ? -ClimberConstants.CONTROL_SPEED : ClimberConstants.CONTROL_SPEED;
        climberMotor.set(speed);
    }

    public void stopClimber() {
        climberMotor.stopMotor();
    }
}
