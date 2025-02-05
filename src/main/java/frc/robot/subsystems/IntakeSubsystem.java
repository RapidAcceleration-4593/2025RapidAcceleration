package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leftExtensionMotor = IntakeConstants.leftExtensionMotor;
    private final SparkMax rightExtensionMotor = IntakeConstants.rightExtensionMotor;

    private final SparkMax leftIntakeMotor = IntakeConstants.leftIntakeMotor;
    private final SparkMax rightIntakeMotor = IntakeConstants.rightIntakeMotor;

    private final DigitalInput leftLimitSwitch = IntakeConstants.leftLimitSwitch;
    private final DigitalInput rightLimitSwitch = IntakeConstants.rightLimitSwitch;

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

    /**
     * Runs the intake motor at the specified speed.
     * @param motor The motor to control.
     * @param speed The speed to set for the motor.
     */
    private void runIntakeMotor(SparkMax motor, double speed) {
        motor.set(speed);
    }
    
    /**
     * Runs the extension motor at the specified speed.
     * @param motor The motor to control.
     * @param speed The speed to set for the motor.
     */
    private void runExtensionMotor(SparkMax motor, double speed) {
        motor.set(speed);
    }

    /**
     * Checks if a given limit switch is pressed.
     * @param limitSwitch The limit switch to check.
     * @return Whether the limit switch is pressed or not.
     */
    private boolean isLimitSwitchPressed(DigitalInput limitSwitch) {
        return !limitSwitch.get();
    }

    /**
     * Checks if a motor is stalling based on its current draw.
     * @param motor The motor to check.
     * @return True if the motor is stalling, false otherwise.
     */
    private boolean isMotorStalling(SparkMax motor) {
        return motor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_AMPERAGE;
    }


    /** ----- Factory Commands ----- */

    // TODO: Create command to check state of intakes.

    /**
     * Runs the left intake motor at the constant intake speed.
     * @return A command to run the left intake motor.
     */
    public Command runLeftIntake() {
        return run(
            () -> runIntakeMotor(
                leftIntakeMotor,
                IntakeConstants.INTAKE_MOTOR_SPEED
            )
        );
    }

    /**
     * Runs the right intake motor at the constant intake speed.
     * @return A command to run the right intake motor.
     */
    public Command runRightIntake() {
        return run(
            () -> runIntakeMotor(
                rightIntakeMotor,
                IntakeConstants.INTAKE_MOTOR_SPEED
            )
        );
    }

    /**
     * Runs the left extension motor inward until the limit switch is pressed.
     * @return A command to run the left extension motor inward.
     */
    public Command runLeftExtensionIn() {
        return run(
            () -> runExtensionMotor(
                leftExtensionMotor,
                -IntakeConstants.EXTENSION_MOTOR_SPEED
            )
        ).until(() -> isLimitSwitchPressed(leftLimitSwitch));
    }

    /**
     * Runs the left extension motor outward until it starts stalling.
     * @return A command to run the left extension motor outward.
     */
    public Command runLeftExtensionOut() {
        return run(
            () -> runExtensionMotor(
                leftExtensionMotor,
                IntakeConstants.EXTENSION_MOTOR_SPEED
            )
        ).until(() -> isMotorStalling(leftExtensionMotor));
    }

    /**
     * Runs the right extension motor inward until the limit switch is pressed.
     * @return A command to run the right extension motor inward.
     */
    public Command runRightExtensionIn() {
        return run(
            () -> runExtensionMotor(
                rightExtensionMotor,
                -IntakeConstants.EXTENSION_MOTOR_SPEED
            )
        ).until(() -> isLimitSwitchPressed(rightLimitSwitch));
    }

    /**
     * Runs the right extension motor outward until it starts stalling.
     * @return A command to run the right extension motor outward.
     */
    public Command runRightExtensionOut() {
        return run(
            () -> runExtensionMotor(
                rightExtensionMotor,
                IntakeConstants.EXTENSION_MOTOR_SPEED
            )
        ).until(() -> isMotorStalling(rightExtensionMotor));
    }
}
