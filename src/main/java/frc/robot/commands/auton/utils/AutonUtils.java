package frc.robot.commands.auton.utils;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonUtils {

    /** SwerveSubsystem Object. */
    public SwerveSubsystem drivebase;

    /** ElevatorSubsystem Object. */
    public ElevatorSubsystem elevatorSubsystem;

    /** ArmSubsystem Object. */
    public ArmSubsystem armSubsystem;

    /** Serializer Object. */
    public SerializerSubsystem serializerSubsystem;

    /** Constructor for AutonUtils. */
    public AutonUtils(SwerveSubsystem drivebase, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SerializerSubsystem serializerSubsystem) {
        this.drivebase = drivebase;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.serializerSubsystem = serializerSubsystem;
    }

    /**
     * Command to reset the robot's odometry to initial pose, adjusted for the current
     * alliance color by flipping it if necessary.
     * @param choreoPath The PathPlannerPath containing the trajectory to use for
     *                   resetting the robot's odometry.
     * @return           A command that, when run, resets the robot's odometry to the
     *                   initial pose of given path.
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
     * @return A Functional Command to set the state of the elevator during autonomous.
     */
    public Command goToElevatorState(ElevatorStates state) {
        return elevatorSubsystem.GoToStateCommand(state);
    }

    /**
     * Functional Command to set arm state while running PID Control.
     * @param state The selected state of the arm.
     * @return A Functional Command to set the state of the arm during autonomous.
     */
    public Command goToArmState(ArmStates state) {
        return armSubsystem.GoToStateCommand(state);
    }

    /**
     * Functional Command to rotate the arm down in Autonomous.
     * @return A lower setpoint for the arm mechanism.
     */
    public Command scoreCoralCommand() {
        return Commands.race(
            new FunctionalCommand(
                () -> armSubsystem.placeCoralCommand(),
                () -> armSubsystem.controlArmState(),
                interrupted -> armSubsystem.stopMotor(),
                () -> armSubsystem.atSetpoint(),
                armSubsystem
            ),
            new WaitCommand(0.5) // Timeout after 0.5 seconds.
        );
    }

    public Command runSerializerCommand(double seconds) {
        return Commands.sequence(
            Commands.race(
                Commands.run(() -> serializerSubsystem.runSerializer(false), serializerSubsystem),
                Commands.waitSeconds(seconds)
            ),
            Commands.runOnce(() -> serializerSubsystem.stopSerializer())
        );
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
     * Flip a field position to the other side of the field, maintaining a blue alliance origin.
     * @param position The position to flip.
     * @return The flipped position.
     */
    private Translation2d flipFieldPosition(Translation2d position) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - position.getX(), position.getY());
    }

    /**
     * Flip a field rotation to the other side of the field, maintaining a blue alliance origin.
     * @param rotation The rotation to flip.
     * @return The flipped rotation.
     */
    private Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }
    
    /**
     * Flip a field pose to the other side of the field, maintaining a blue alliance origin.
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    public Pose2d flipFieldPose(Pose2d pose) {
        return new Pose2d(flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
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

    /** Flip Pose2d locations on field for Coral Station on red alliance. */
    private Pose2d[] flipFieldPoses(Pose2d[] bluePoses) {
        Pose2d[] redPoses = new Pose2d[bluePoses.length];
        for (int i = 0; i < bluePoses.length; i++) {
            redPoses[i] = flipFieldPose(bluePoses[i]);
        }
        return redPoses;
    }
}