package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakePIDConstants;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.utils.ControlSubsystem;

public class IntakeSubsystem extends ControlSubsystem<IntakeStates> {
    
    private final SparkMax leaderDeployMotor = IntakeConstants.leaderDeployMotor;
    private final SparkMax followerDeployMotor = IntakeConstants.followerDeployMotor;

    private final SparkMax innerIntakeMotor = IntakeConstants.innerIntakeMotor;
    private final SparkMax outerIntakeMotor = IntakeConstants.outerIntakeMotor;

    private static final double[] SETPOINTS = {0, 1000, 3200};

    private final SparkMaxConfig brakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig coastConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();
    private final SparkMaxConfig encoderConfig = new SparkMaxConfig();

    private double lastCurrent = 0;

    public IntakeSubsystem() {
        super(
            new ProfiledPIDController(
                IntakePIDConstants.INTAKE_PID.kP,
                IntakePIDConstants.INTAKE_PID.kI,
                IntakePIDConstants.INTAKE_PID.kD,
                new TrapezoidProfile.Constraints(
                    IntakePIDConstants.MAX_VELOCITY,
                    IntakePIDConstants.MAX_ACCELERATION
                )
            )
        );

        brakeConfig.idleMode(IdleMode.kBrake);
        coastConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(leaderDeployMotor, true);
        encoderConfig.alternateEncoder.countsPerRevolution(1).setSparkMaxDataPortConfig();

        leaderDeployMotor.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerDeployMotor.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        innerIntakeMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outerIntakeMotor.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderDeployMotor.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerDeployMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.setTolerance(IntakePIDConstants.TOLERANCE);
        controller.reset(0);
        resetEncoder();
    }

    @Override
    public double getStateSetpoint(IntakeStates state) {
        return switch (state) {
            case IN -> SETPOINTS[0];
            case L1 -> SETPOINTS[1];
            case OUT -> SETPOINTS[2];
            default -> throw new Error("Passed in an IntakeState that does not have an associated setpoint!");
        };
    }

    @Override
    public IntakeStates getCurrentState() {
        if (getEncoderValue() <= SETPOINTS[0] + 500) {
            return IntakeStates.IN;
        } else if (getEncoderValue() <= SETPOINTS[1] + 500) {
            return IntakeStates.L1;
        } else {
            return IntakeStates.OUT;
        }
    }

    @Override
    public void controlStates() {
        updateValues();

        if (isManualControlEnabled()) {
            return;
        }

        if (shouldCoast()) {
            stopMotors();
        } else {
            controlOutput();
        }
    }

    @Override
    public void controlOutput() {
        double output = controller.calculate(getEncoderValue(), getSetpoint());
        setMotorSpeeds(output);
    }

    /**
     * Checks if the current state is IN or OUT.
     * @return Whether the intake deployment motors should coast to the setpoint.
     */
    private boolean shouldCoast() {
        return (getCurrentState() == IntakeStates.IN) ||
               (getCurrentState() == IntakeStates.OUT);
    }

    /**
     * Checks if the current spike is detected on the outer intake motor.
     * @return Whether the current spike is detected.
     */
    public boolean isCurrentSpikeDetected() {
        double current = outerIntakeMotor.getOutputCurrent();

        if (Math.abs(current - lastCurrent) > IntakeConstants.SPIKE_CURRENT) {
            lastCurrent = current;
            return true;
        }

        lastCurrent = current;
        return false;
    }

    /**
     * Sets the motor speeds for the intake deployment.
     * @param speed The speed of the deploy intake motors.
     */
    public void setMotorSpeeds(double speed) {
        leaderDeployMotor.set(-speed);
    }

    /**
     * Stops the intake deploy motors.
     */
    public void stopMotors() {
        leaderDeployMotor.stopMotor();
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

    @Override
    public double getEncoderValue() {
        return leaderDeployMotor.getAlternateEncoder().getPosition();
    }

    @Override
    public void resetEncoder() {
        leaderDeployMotor.getAlternateEncoder().setPosition(0);
    }

    @Override
    protected void updateValues() {
        SmartDashboard.putNumber("I-Encoder", getEncoderValue());
    }
}
