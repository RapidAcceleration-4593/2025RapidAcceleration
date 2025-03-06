package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    SparkMax climberMotor = ClimberConstants.climberMotor;

    private final SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

    /**
     * Creates a new ClimberSubsystem and configures its motors.
     */
    public ClimberSubsystem() {
        climbMotorConfig.idleMode(IdleMode.kBrake);
        climberMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor speed.
     * @param speed
     */
    public void setMotorSpeed(float speed) {
        climberMotor.set(speed);
    }

    //TODO: determine if running the motor up makes the climber move up.
    public Command moveClimberUpCommand() {
        return run(() -> setMotorSpeed(ClimberConstants.CLIMBER_MOVE_SPEED)).finallyDo((interrupted) -> setMotorSpeed(0));
    }

    public Command moveClimberDownCommand() {
        return run(() -> setMotorSpeed(-ClimberConstants.CLIMBER_MOVE_SPEED)).finallyDo((interupted) -> setMotorSpeed(0));
    }
}
