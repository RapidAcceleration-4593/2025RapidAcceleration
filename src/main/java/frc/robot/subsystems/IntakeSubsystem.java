package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeStates;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax leftExtensionMotor = IntakeConstants.leftExtensionMotor;
    private final SparkMax rightExtensionMotor = IntakeConstants.rightExtensionMotor;

    private final SparkMax leftIntakeMotor = IntakeConstants.leftIntakeMotor;
    private final SparkMax rightIntakeMotor = IntakeConstants.rightIntakeMotor;

    private IntakeStates leftState = IntakeStates.RETRACTED_STOPPED;
    private IntakeStates rightState = IntakeStates.RETRACTED_STOPPED;

    private SparkMaxConfig config = new SparkMaxConfig();

    /**
     * Constructor for the IntakeSubsystem class.
     * Initializes the motor configuration.
     */
    public IntakeSubsystem() {
        config.idleMode(IdleMode.kBrake);

        leftIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        leftExtensionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightExtensionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Intake State System ----- */

    /** Controls the intake states based on the current setpoints. */
    public void controlIntakeStates() {
        leftState = handleState(leftExtensionMotor, leftIntakeMotor, leftState, 1);
        rightState = handleState(rightExtensionMotor, rightIntakeMotor, rightState, -1);
    }

    /** Sets the state of the left intake. */
    public void setLeftState(IntakeStates state) {
        leftState = state;
    }

    /** Sets the state of the right intake. */
    public void setRightState(IntakeStates state) {
        rightState = state;
    }


    /** ----- Intake State Handling ----- */

    /**
     * Handles the possible states of each intake.
     * @param extensionMotor The motor used for extension.
     * @param intakeMotor The motor used for intaking.
     * @param state The left or right state of the intake.
     */
    private IntakeStates handleState(SparkMax extensionMotor, SparkMax intakeMotor, IntakeStates state, int inversion) {
        switch (state) {
            case EXTENDING:
                extensionMotor.set(IntakeConstants.EXTENSION_MOTOR_SPEED);
                if (extensionMotor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_LIMIT) {
                    extensionMotor.stopMotor();
                    return IntakeStates.EXTENDED_STOPPED;
                }
                break;
            case EXTENDED_RUNNING:
                if (intakeMotor.get() != IntakeConstants.INTAKE_MOTOR_SPEED * inversion) {
                    intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
                }
                break;
            case EXTENDED_RUNNING_REVERSE:
                if (intakeMotor.get() != -IntakeConstants.INTAKE_MOTOR_SPEED * inversion) {
                    intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
                }
                break;
            case EXTENDED_STOPPED:
                intakeMotor.stopMotor();
                break;
            case RETRACTING:
                extensionMotor.set(-IntakeConstants.EXTENSION_MOTOR_SPEED);
                if (extensionMotor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_LIMIT) {
                    extensionMotor.stopMotor();
                    return IntakeStates.RETRACTED_STOPPED;
                }
                break;
            case RETRACTED_STOPPED:
                extensionMotor.stopMotor();
                intakeMotor.stopMotor();
                break;
        }
        return state;
    }


    /** ----- Intake State Checks ----- */

    /**
     * Checks if the left intake is extended.
     * @returns If the left intake is extended.
     */
    public boolean isLeftExtended() {
        return leftState.toString().contains("EXTENDED");
    }

    /** 
     * Checks if the right intake is extended.
     * @returns If the right intake is extended.
     */
    public boolean isRightExtended() {
        return rightState.toString().contains("EXTENDED");
    }


    /** ----- Factory Command Methods ----- */

    /** Toggles the left intake between extended/retracted. */
    public void toggleLeftIntakeCommand() {
        if (isLeftExtended()) {
            setLeftState(IntakeStates.RETRACTING);
        } else {
            setLeftState(IntakeStates.EXTENDING);
        }
    }

    /** Toggles the right intake between extended/retracted. */
    public void toggleRightIntakeCommand() {
        if (isRightExtended()) {
            setRightState(IntakeStates.RETRACTING);
        } else {
            setRightState(IntakeStates.EXTENDING);
        }
    }


    /** ----- Temporary Manual Control ----- */

    public void runLeftIntake(boolean reversed) {
        double speed = reversed ? -IntakeConstants.INTAKE_MOTOR_SPEED : IntakeConstants.INTAKE_MOTOR_SPEED;
        leftIntakeMotor.set(speed);
    }

    public void runRightIntake(boolean reversed) {
        double speed = reversed ? IntakeConstants.INTAKE_MOTOR_SPEED : -IntakeConstants.INTAKE_MOTOR_SPEED;
        rightIntakeMotor.set(speed);
    }

    public void stopLeftIntake() {
        leftIntakeMotor.stopMotor();
    }

    public void stopRightIntake() {
        rightIntakeMotor.stopMotor();
    }

    public void extendLeftIntake(boolean retract) {
        SmartDashboard.putNumber("L-EOutput", leftExtensionMotor.getOutputCurrent());
        double speed = retract ? -IntakeConstants.EXTENSION_MOTOR_SPEED : IntakeConstants.EXTENSION_MOTOR_SPEED;
        // if (leftExtensionMotor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_LIMIT) {
        //     stopLeftExtension();
        // } else {
            leftExtensionMotor.set(speed);
        // }
    }

    public void extendRightIntake(boolean retract) {
        SmartDashboard.putNumber("R-EOutput", rightExtensionMotor.getOutputCurrent());
        double speed = retract ? IntakeConstants.EXTENSION_MOTOR_SPEED : -IntakeConstants.EXTENSION_MOTOR_SPEED;
        // if (rightExtensionMotor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_LIMIT) {
        //     stopRightExtension();
        // } else {
            rightExtensionMotor.set(speed);
        // }
    }

    public void stopLeftExtension() {
        leftExtensionMotor.stopMotor();
    }

    public void stopRightExtension() {
        rightExtensionMotor.stopMotor();
    }
}
