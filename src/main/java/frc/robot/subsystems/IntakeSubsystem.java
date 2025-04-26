package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakePIDConstants;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.control.ControlSubsystem;

public class IntakeSubsystem extends ControlSubsystem<IntakeStates> {
    
    private final SparkMax leaderDeployMotor = IntakeConstants.leaderDeployMotor;
    private final SparkMax followerDeployMotor = IntakeConstants.followerDeployMotor;

    private final SparkMax innerIntakeMotor = IntakeConstants.innerIntakeMotor;
    private final SparkMax outerIntakeMotor = IntakeConstants.outerIntakeMotor;

    private static final double[] SETPOINTS = {0, 0, 0};

    private final Timer stallTimer = new Timer();

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

        applyBrakeConfig(List.of(leaderDeployMotor, followerDeployMotor, innerIntakeMotor, outerIntakeMotor));
        applyFollowerConfig(leaderDeployMotor, followerDeployMotor, true);
    
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
        return currentState;
    }

    @Override
    public void controlStates() {
        updateValues();

        if (isManualControlEnabled()) {
            return;
        }

        if (isMotorStalled()) {
            stopMotors();
            setSetpoint(getEncoderValue());
        } else {
            controlOutput();
        }
    }

    private boolean isMotorStalled() {
        if (leaderDeployMotor.getOutputCurrent() < IntakeConstants.STALL_CURRENT) {
            stallTimer.reset();
            return false;
        }

        if (!stallTimer.isRunning()) {
            stallTimer.start();
        }

        return stallTimer.hasElapsed(0.5);
    }

    public void setMotorSpeeds(double speed) {
        leaderDeployMotor.set(speed);
    }

    public void setIntakeSpeed(double inner, double outer) {
        innerIntakeMotor.set(-inner);
        outerIntakeMotor.set(outer);
    }

    public void stopMotors() {
        leaderDeployMotor.stopMotor();
    }

    public void stopIntake() {
        innerIntakeMotor.stopMotor();
        outerIntakeMotor.stopMotor();
    }

    @Override
    public double getEncoderValue() {
        // return leaderDeployMotor.getAlternateEncoder().getPosition();
        return 0;
    }

    @Override
    public void resetEncoder() {
        // leaderDeployMotor.getAlternateEncoder().setPosition(0);
    }

    @Override
    protected void updateValues() {
        SmartDashboard.putNumber("I-Encoder", getEncoderValue());
        SmartDashboard.putBoolean("I-Stalled", isMotorStalled());
    }
}
