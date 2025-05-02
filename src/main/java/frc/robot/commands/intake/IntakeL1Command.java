package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class IntakeL1Command extends Command {
    
    private final IntakeDeploySubsystem intakeDeploySubsystem;
    private final IntakeFeederSubsystem intakeFeederSubsystem;

    public IntakeL1Command(IntakeDeploySubsystem deploySubsystem, IntakeFeederSubsystem feederSubsystem) {
        this.intakeDeploySubsystem = deploySubsystem;
        this.intakeFeederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        intakeFeederSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SPEED);
        intakeDeploySubsystem.setControlState(IntakeStates.OUT);
    }

    @Override
    public void end(boolean interrupted) {
        intakeFeederSubsystem.stopIntake();
        intakeDeploySubsystem.setControlState(IntakeStates.L1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
