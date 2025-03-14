package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreCoralCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private boolean originallyManual;
    private double encoderTarget = 0;

    private static final double SCORE_DISTANCE = -300;
    private static final double MOTOR_SPEED = 0.3;
    private static final double DEADBAND = 20;

    public ScoreCoralCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        originallyManual = armSubsystem.isManualControlEnabled();
        encoderTarget = armSubsystem.getEncoderValue() + SCORE_DISTANCE;

        armSubsystem.setManualControl(true);
        armSubsystem.setMotorSpeed(MOTOR_SPEED * ((SCORE_DISTANCE > 0) ? 1 : -1));
    }

    @Override
    public boolean isFinished() {
        // In deadband or limit switch pressed
        return Math.abs(armSubsystem.getEncoderValue() - encoderTarget) < DEADBAND || isLSPressed();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotorSpeed(0);
        armSubsystem.setManualControl(originallyManual);
    }

    private boolean isLSPressed() {
        if (SCORE_DISTANCE > 0) {
            return armSubsystem.isTopLimitSwitchPressed();
        }
        return armSubsystem.isBottomLimitSwitchPressed();
    }
}
