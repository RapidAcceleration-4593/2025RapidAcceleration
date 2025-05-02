package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeFeederSubsystem extends SubsystemBase {
    
    private final SparkMax innerIntakeMotor = IntakeConstants.RIGHT_INTAKE_MOTOR;
    private final SparkMax outerIntakeMotor = IntakeConstants.LEFT_INTAKE_MOTOR;

    private final SparkMaxConfig brakeConfig = new SparkMaxConfig();

    public IntakeFeederSubsystem() {
        brakeConfig.idleMode(IdleMode.kBrake);

        innerIntakeMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outerIntakeMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the speed of the intake motors.
     * @param inner The speed of the inner intake motor.
     * @param outer The speed of the outer intake motor.
     */
    public void setIntakeSpeed(double inner, double outer) {
        innerIntakeMotor.set(-inner);
        outerIntakeMotor.set(outer);
    }

    /**
     * Stops the intake motors.
     */
    public void stopIntake() {
        innerIntakeMotor.stopMotor();
        outerIntakeMotor.stopMotor();
    }
}
