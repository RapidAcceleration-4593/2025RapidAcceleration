package frc.robot.commands.auton.utils;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.arm.AdjustArmCommand;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.armivator.SetArmivatorState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonUtils {

    /** ElevatorSubsystem Object. */
    public final ElevatorSubsystem elevatorSubsystem;

    /** ArmSubsystem Object. */
    public final ArmSubsystem armSubsystem;

    /** Serializer Object. */
    public final SerializerSubsystem serializerSubsystem;

    /** SwerveSubsystem Object. */
    public final SwerveSubsystem drivebase;

    /** Constructor for AutonUtils. */
    public AutonUtils(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SerializerSubsystem serializerSubsystem, SwerveSubsystem drivebase) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.serializerSubsystem = serializerSubsystem;
        this.drivebase = drivebase;
    }

    /**
     * Command to reset the robot's odometry to initial pose, adjusted for the current
     * alliance color by flipping it if necessary.
     * @param path  The PathPlannerPath containing the trajectory to use for
     *              resetting the robot's odometry.
     * @return      A command that, when run, resets the robot's odometry to the
     *              initial pose of given path.
     */
    public Command resetOdometry(PathPlannerPath path) {
        return drivebase.runOnce(
            () -> {
                RobotConfig config = getRobotConfig();

                Pose2d pose = path
                    .generateTrajectory(new ChassisSpeeds(), new Rotation2d(Math.PI), config)
                    .getInitialPose();
                    

                if (drivebase.isRedAlliance()) {
                    pose = FlippingUtil.flipFieldPose(pose);
                }
    
                drivebase.resetOdometry(pose);
        });
    }

    /**
     * Functional Command to set elevator state while running PID Control.
     * @param state The selected state of the elevator.
     * @param timeout The time to execute before moving on.
     * @return A Functional Command to set the state of the elevator during autonomous.
     */
    public Command setElevatorState(ElevatorStates state, double timeout) {
        return new SetElevatorState(elevatorSubsystem, state).withTimeout(timeout);
    }

    /**
     * Command to set arm state while running PID Control.
     * @param state The selected state of the arm.
     * @param timeout The time to execute before moving on.
     * @return A Functional Command to set the state of the arm during autonomous.
     */
    public Command setArmState(ArmStates state, double timeout) {
        return new SetArmState(armSubsystem, state).withTimeout(timeout);
    }

    /**
     * Command to set elevator and arm state while running PID Control.
     * @param elevatorState The selected state of the elevator.
     * @param armState The selected state of the arm.
     * @return A Functional Command to set the state of the elevator and arm during autonomous.
     */
    public Command setArmivatorState(ElevatorStates elevatorState, ArmStates armState) {
        return new SetArmivatorState(elevatorSubsystem, armSubsystem, elevatorState, armState);
    }

    /**
     * Command to rotate the arm down in Autonomous.
     * @return A lower setpoint for the arm mechanism.
     */
    public Command scoreCoralCommand(double timeout) {
        return new AdjustArmCommand(armSubsystem, -ArmConstants.PLACE_ROTATION_AMOUNT).withTimeout(timeout);

    }

    /**
     * Command to run the serializer for a set amount of time.
     * @param timeout The amount of time to run the serializer.
     * @return A Functional Command to run the serializer for a set amount of time.
     */
    public Command runSerializerCommand(double timeout) {
        return serializerSubsystem.runSerializerCommand().withTimeout(timeout);
    }

    /**
     * Command to drive backward to end autonomous.
     * @return A Deferred Command to drive backward from the current pose.
     */
    public Command driveBackward() {
        return drivebase.driveToDistance(-0.75);
    }

    /**
     * Load the PathPlanner trajectory file to path.
     * @param pathName Name of the path.
     * @return PathPlanner Path.
     */
    public PathPlannerPath loadPath(String pathName) {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load path: " + pathName, e);
        }
    }

    /**
     * Retrieves the robot configuration from Deploy Settings.
     * @return The RobotConfig instance generated by PathPlanner.
     * @throws RuntimeException If failed to retrieve configuration.
     */
    public RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to retrieve RobotConfig from Deploy Settings.", e);
        }
    }

    /** Pose2d for Coral Station on bottom of blue alliance. */
    public final Pose2d[] BLUE_BOTTOM_CHUTE = {
        new Pose2d(0.708291, 1.303966, Rotation2d.fromDegrees(54)),
        new Pose2d(1.117988, 1.006304, Rotation2d.fromDegrees(54)),
        new Pose2d(1.527684, 0.708642, Rotation2d.fromDegrees(54))
    };

    /** Pose2d for Coral Station on top of blue alliance. */
    public final Pose2d[] BLUE_TOP_CHUTE = {
        new Pose2d(0.708291, 6.747834, Rotation2d.fromDegrees(-54)),
        new Pose2d(1.117988, 7.045496, Rotation2d.fromDegrees(-54)),
        new Pose2d(1.527684, 7.343158, Rotation2d.fromDegrees(-54))
    };

    /** Pose2d for Coral Station on bottom of red alliance. */
    public final Pose2d[] RED_BOTTOM_CHUTE = flipFieldPoses(BLUE_BOTTOM_CHUTE);

    /** Pose2d for Coral Station on top of red alliance. */
    public final Pose2d[] RED_TOP_CHUTE = flipFieldPoses(BLUE_TOP_CHUTE);

    /** Flip Pose2d locations on field for Coral Station (aka Acceleration Station) on red alliance. */
    private Pose2d[] flipFieldPoses(Pose2d[] bluePoses) {
        Pose2d[] redPoses = new Pose2d[bluePoses.length];
        for (int i = 0; i < bluePoses.length; i++) {
            redPoses[i] = FlippingUtil.flipFieldPose(bluePoses[i]);
        }
        return redPoses;
    }
}