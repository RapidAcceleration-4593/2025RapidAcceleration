package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class IntakeL1Command extends Command {
    
    private final IntakeDeploySubsystem intakeDeploySubsystem;
    private final IntakeFeederSubsystem intakeFeederSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;

    public IntakeL1Command(IntakeDeploySubsystem deploySubsystem, IntakeFeederSubsystem feederSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.intakeDeploySubsystem = deploySubsystem;
        this.intakeFeederSubsystem = feederSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(feederSubsystem, deploySubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setControlState(ElevatorStates.PICKUP);
        armSubsystem.setControlState(ArmStates.BOTTOM);
        intakeDeploySubsystem.setControlState(IntakeStates.OUT);
        intakeFeederSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SPEED);
    }

    @Override
    public void execute() {
        elevatorSubsystem.controlStates();
        armSubsystem.controlStates();
        intakeDeploySubsystem.controlStates();
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
