package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakePIDConstants;
import frc.robot.Constants.RobotStates.IntakeStates;


public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax innerIntakeMotor = IntakeConstants.innerIntakeMotor;
    private final SparkMax outerIntakeMotor = IntakeConstants.outerIntakeMotor;

    private final SparkMax leaderDeployMotor = IntakeConstants.leaderDeployMotor;
    private final SparkMax followerDeployMotor = IntakeConstants.followerDeployMotor;

    private final Encoder intakeEncoder = IntakeConstants.intakeEncoder;

    private final PIDController intakePID = new PIDController(IntakePIDConstants.INTAKE_PID.kP,
                                                              IntakePIDConstants.INTAKE_PID.kI,
                                                              IntakePIDConstants.INTAKE_PID.kD);

    private static final double[] SETPOINTS = {0, 100, 200};

    private final SparkMaxConfig brakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    /**
     * Returns if the elevator is solely in manual mode.
     * PID is completely disabled. Access only through {@link #isManualControlEnabled()}.
     */
    private boolean manualControlEnabled = false;

    /**
     * Constructor for the IntakeSubsystem class.
     * Configures the motor settings and sets the idle mode to brake.
     */
    public IntakeSubsystem() {
        brakeConfig.idleMode(IdleMode.kBrake);
        followerConfig.follow(leaderDeployMotor, true);

        innerIntakeMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outerIntakeMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leaderDeployMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerDeployMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerDeployMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakePID.setTolerance(IntakePIDConstants.TOLERANCE);
    }
    

    /** ----- Intake State Management ----- */

    /**
     * Retrieves the setpoint for the specified IntakeStates.
     * @param state The desired intake position.
     * @return The {@link IntakeSubsystem#SETPOINTS} value corresponding to the state.
     */
    private double getIntakeState(IntakeStates state) {
        return switch (state) {
            case IN -> SETPOINTS[0];
            case L1 -> SETPOINTS[1];
            case OUT -> SETPOINTS[2];
            default -> throw new Error("Passed in an IntakeState that does not have an associated setpoint!");
        };
    }

    /**
     * Sets the setpoint of the intake to the target state.
     * @param state The desired intake position.
     */
    public void setIntakeState(IntakeStates state) {
        intakePID.setSetpoint(getIntakeState(state));
    }


    /** ----- Intake State System ----- */

    /**
     * Controls the intake state based on the current setpoint and encoder value.
     * If manual control is enabled, the method returns without making any changes.
     * If the motor is stalled, the deploy motor is stopped and the setpoint is updated.
     */
    public void controlIntakeState() {
        updateValues();

        if (isManualControlEnabled()) {
            return;
        }

        if (isMotorStalled()) {
            stopDeploy();
            setSetpoint(getEncoderValue());
        } else {
            controlIntake();
        }
    }

    /**
     * Controls the intake motor using a PID controller.
     * The output is calculated based on the encoder value and the setpoint.
     */
    private void controlIntake() {
        double output = intakePID.calculate(getEncoderValue());

        if (atSetpoint()) {
            stopDeploy();
        } else {
            setDeploySpeed(output);
        }
    }
    

    /** ----- Motor and Encoder Abstraction ----- */

    /**
     * Checks if the motor is stalled by comparing the encoder rate to the motor speed.
     * @return True if the motor is stalled, false otherwise.
     */
    private boolean isMotorStalled() {
        return intakeEncoder.getRate() != leaderDeployMotor.get();
    }

    /**
     * Retrieves the current encoder value of the intake.
     * @return The current encoder value of the intake.
     */
    public double getEncoderValue() {
        return intakeEncoder.get();
    }

     /**
     * Sets the setpoint for the intake PID controller.
     * @param setpoint New setpoint value.
     */
    public void setSetpoint(double setpoint) {
        intakePID.setSetpoint(setpoint);
    }

    /**
     * Whether the intake is at its setpoint.
     * @return If the intake is at the setpoint, accounting for tolerance.
     */
    private boolean atSetpoint() {
        return intakePID.atSetpoint();
    }


    /** ----- Factory Command Methods ----- */

    /**
     * Sets the speed of the deploy motor.
     * @param speed The speed to set the deploy motor to.
     */
    public void setDeploySpeed(double speed) {
        leaderDeployMotor.set(speed);
    }

    /**
     * Sets the speed of the intake motors.
     * @param outer The speed to set the outer intake motor to.
     * @param inner The speed to set the inner intake motor to.
     */
    public void setIntakeSpeed(double outer, double inner) {
        outerIntakeMotor.set(-outer);
        innerIntakeMotor.set(-inner);
    }

    /** Stops the deploy motors. */
    public void stopDeploy() {
        leaderDeployMotor.stopMotor();
    }

    /** Stops the intake motors. */
    public void stopIntake() {
        innerIntakeMotor.stopMotor();
        outerIntakeMotor.stopMotor();
    }

    /**
     * Sets the speed of the intake motors to a specific value.
     * @param enabled The speed to set the intake motors to.
     */
    public void setManualControl(boolean enabled) {
        manualControlEnabled = enabled;
    }

    /**
     * Checks if manual control is enabled.
     * @return True if manual control is enabled, false otherwise.
     */
    public boolean isManualControlEnabled() {
        return manualControlEnabled;
    }

    /** Updates values to SmartDashboard/ShuffleBoard. */
    private void updateValues() {
        SmartDashboard.putNumber("I-Encoder", getEncoderValue());
    }
}
