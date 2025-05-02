package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakePIDConstants;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.utils.RegularControlSubsystem;

public class IntakeDeploySubsystem extends RegularControlSubsystem<IntakeStates> {
    
    private static final SparkMax leaderDeployMotor = IntakeConstants.LEFT_DEPLOY_MOTOR;
    private static final SparkMax followerDeployMotor = IntakeConstants.RIGHT_DEPLOY_MOTOR;

    private static final double[] SETPOINTS = {0, 1300, 3200};

    private final SparkMaxConfig coastConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();
    private final SparkMaxConfig encoderConfig = new SparkMaxConfig();

    public IntakeDeploySubsystem() {
        super(
            new PIDController(
                IntakePIDConstants.INTAKE_PID.kP,
                IntakePIDConstants.INTAKE_PID.kI,
                IntakePIDConstants.INTAKE_PID.kD
            )
        );

        coastConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(leaderDeployMotor, true);
        encoderConfig.alternateEncoder.countsPerRevolution(1).setSparkMaxDataPortConfig();

        leaderDeployMotor.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerDeployMotor.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderDeployMotor.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerDeployMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.setTolerance(IntakePIDConstants.TOLERANCE);
        resetEncoder();
    }

    @Override
    public double getStateSetpoint(IntakeStates state) {
        return switch (state) {
            case IN -> SETPOINTS[0];
            case L1 -> SETPOINTS[1];
            case OUT -> SETPOINTS[2];
            default -> throw new IllegalArgumentException("Passed in an IntakeState that does not have an associated setpoint!");
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
