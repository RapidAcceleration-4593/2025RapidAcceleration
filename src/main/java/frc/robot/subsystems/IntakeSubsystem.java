package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeSides;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leftIntakeMotor = IntakeConstants.leftIntakeMotor;
    private final SparkMax rightIntakeMotor = IntakeConstants.rightIntakeMotor;

    private SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the IntakeSubsystem class.
     * Initializes the motor configuration.
     */
    public IntakeSubsystem() {
        config.idleMode(IdleMode.kBrake);

        leftIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Factory Command Methods ----- */

    /**
     * Runs a specified intake side, either forward or backward.
     * @param side The side of the intake to run.
     * @param inverted Whether the intake should spin reversely.
     */
    public void runIntake(IntakeSides side, boolean inverted) {
        switch (side) {
            case LEFT:
                leftIntakeMotor.set(inverted ? -IntakeConstants.CONTROL_SPEED : IntakeConstants.CONTROL_SPEED);
                break;
            case RIGHT:
                rightIntakeMotor.set(inverted ? IntakeConstants.CONTROL_SPEED : -IntakeConstants.CONTROL_SPEED);
                break;
        }
    }

    /**
     * Stops a specified intake side.
     * @param side The side of the intake to stop.
     */
    public void stopIntake(IntakeSides side) {
        switch (side) {
            case LEFT:
                leftIntakeMotor.stopMotor();
                break;
            case RIGHT:
                rightIntakeMotor.stopMotor();
                break;
        }
    }
}
