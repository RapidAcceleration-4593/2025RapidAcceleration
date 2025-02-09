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
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.MOTOR_STALL_LIMIT);

        leftIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        leftExtensionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightExtensionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** ----- Intake State System ----- */

    /** Controls the intake states based on the current setpoints. */
    public void manageIntakeStates() {
        SmartDashboard.putNumber("LE-Current", leftExtensionMotor.getOutputCurrent());
        SmartDashboard.putNumber("RE-Current", rightExtensionMotor.getOutputCurrent());
        // handleState(leftExtensionMotor, leftIntakeMotor, leftState);
        // handleState(rightExtensionMotor, rightIntakeMotor, rightState);
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
    @SuppressWarnings("unused")
    private void handleState(SparkMax extensionMotor, SparkMax intakeMotor, IntakeStates state) {
        switch (state) {
            case EXTENDING:
                extensionMotor.set(IntakeConstants.EXTENSION_MOTOR_SPEED);
                if (extensionMotor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_LIMIT) {
                    extensionMotor.stopMotor();
                    state = IntakeStates.EXTENDED_STOPPED;
                }
                break;
            case EXTENDED_RUNNING:
                intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
                break;
            case EXTENDED_RUNNING_REVERSE:
                intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED);
                break;
            case EXTENDED_STOPPED:
                intakeMotor.stopMotor();
                break;
            case RETRACTING:
                extensionMotor.set(-IntakeConstants.EXTENSION_MOTOR_SPEED);
                if (extensionMotor.getOutputCurrent() > IntakeConstants.MOTOR_STALL_LIMIT) {
                    extensionMotor.stopMotor();
                    state = IntakeStates.RETRACTED_STOPPED;
                }
                break;
            case RETRACTED_STOPPED:
                extensionMotor.stopMotor();
                intakeMotor.stopMotor();
                break;
        }
    }


    /** ----- Intake State Checks ----- */

    /**
     * Checks if the left intake is extended.
     * @returns If the left intake is extended.
     */
    public boolean isLeftExtended() {
        return leftState == IntakeStates.EXTENDED_RUNNING ||
               leftState == IntakeStates.EXTENDED_RUNNING_REVERSE ||
               leftState == IntakeStates.EXTENDED_STOPPED;
    }

    /** 
     * Checks if the right intake is extended.
     * @returns If the right intake is extended.
     */
    public boolean isRightExtended() {
        return rightState == IntakeStates.EXTENDED_RUNNING ||
               rightState == IntakeStates.EXTENDED_RUNNING_REVERSE ||
               rightState == IntakeStates.EXTENDED_STOPPED;
    }

    /**
     * Checks if the left intake is running.
     * @return If the left intake is running.
     */
    public boolean isLeftIntakeRunning() {
        return leftState == IntakeStates.EXTENDED_RUNNING;
    }
    
    /**
     * Checks if the right intake is running.
     * @return If the right intake is running.
     */
    public boolean isRightIntakeRunning() {
        return rightState == IntakeStates.EXTENDED_RUNNING;
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

    public void runRightIntake() {
        rightIntakeMotor.set(-0.75);
    }

    public void stopRightIntake() {
        rightIntakeMotor.stopMotor();
    }

    public void runLeftIntake() {
        leftIntakeMotor.set(0.75);
    }

    public void stopLeftIntake() {
        leftIntakeMotor.stopMotor();
    }


    public void runLeftExtensionForward() {
        leftExtensionMotor.set(0.75);
    }

    public void runLeftExtensionBackward() {
        leftExtensionMotor.set(-0.75);
    }

    public void stopLeftExtension() {
        leftExtensionMotor.stopMotor();
    }


    public void runRightExtensionForward() {
        rightExtensionMotor.set(0.75);
    }

    public void runRightExtensionBackward() {
        rightExtensionMotor.set(-0.75);
    }

    public void stopRightExtension() {
        rightExtensionMotor.stopMotor();
    }
}
