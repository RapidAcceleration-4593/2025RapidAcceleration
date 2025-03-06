package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class HandleDashboardSelection extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private int dashboardValue;

    public HandleDashboardSelection(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(elevatorSubsystem, armSubsystem);
    }

    @Override
    public void execute() {
        dashboardValue = (int) SmartDashboard.getNumber("TargetArmivatorState", 1);
        
        switch (dashboardValue) {
            case 1:
                elevatorSubsystem.goToStateCommand(ElevatorStates.BOTTOM);
                armSubsystem.goToStateCommand(ArmStates.BOTTOM);
                break;
            case 2:
                elevatorSubsystem.goToStateCommand(ElevatorStates.BOTTOM);
                armSubsystem.goToStateCommand(ArmStates.L2);
                break;
            case 3:
                elevatorSubsystem.goToStateCommand(ElevatorStates.BOTTOM);
                armSubsystem.goToStateCommand(ArmStates.TOP);
                break;
            case 4:
                elevatorSubsystem.goToStateCommand(ElevatorStates.TOP);
                armSubsystem.goToStateCommand(ArmStates.TOP);
                break;
            default:
                new Error("Invalid Dashboard Selection!");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
