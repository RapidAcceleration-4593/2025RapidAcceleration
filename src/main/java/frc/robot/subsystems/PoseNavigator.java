package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.AutonConstants.DashboardAlignment;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.armivator.ArmivatorCommands;

public class PoseNavigator extends SubsystemBase {

    /** ArmivatorCommands Object. */
    public final ArmivatorCommands armivatorCommands;

    /** SwerveSubsystem Class Object. */
    private final SwerveSubsystem drivebase;

    /** Notifier for Custom Dashboard. */
    private final Notifier dashboardNotifier;

    /** AprilTag field layout. */
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    /**
     * Constructor for the PoseNavigator class.
     * Initializes the notifier that updates the SmartDashboard periodically.
     */
    public PoseNavigator(ArmivatorCommands armivatorCommands, SwerveSubsystem drivebase) {
        this.armivatorCommands = armivatorCommands;
        this.drivebase = drivebase;

        dashboardNotifier = new Notifier(this::updateDashboard);
        dashboardNotifier.startPeriodic(0.2); // Run periodically, 200ms.
    }

    /**
     * Periodically updates the target pose and match time on the SmartDashboard.
     * This method is called by the Notifier.
     */
    private void updateDashboard() {
        SmartDashboard.putNumber("MatchTime", (int) DriverStation.getMatchTime());
        SmartDashboard.putBoolean("ManualControl", armivatorCommands.isManualControlEnabled());
    }

    /**
     * Calculates the reef branch offsets and returns the specific target pose.
     * @param targetID The target pose ID (1-12).
     * @return A {@link Pose2d} corresponding to the given target ID.
     */
    public Pose2d calculateReefPose(int targetID, double distance) {
        List<Pose2d> poses = new ArrayList<>();

        // Start with tag 18, then proceed in counterclockwise order (18 -> 17 -> 22 -> 21 -> 20 -> 19).
        int[] tagOrder = {18, 17, 22, 21, 20, 19};

        // Iterate through each AprilTag in order.
        for (int tagID : tagOrder) {
            aprilTagFieldLayout.getTagPose(tagID).ifPresent(tagPose3d -> {
                Pose2d tagPose = tagPose3d.toPose2d();
                Rotation2d tagRotation = tagPose.getRotation();

                // Iterate over the branch offsets (-1 for left, +1 for right).
                for (double offset : new double[]{-1, 1}) {
                    // Calculate the branch translation and extrusion from face.
                    Translation2d branchTranslation = tagPose.getTranslation()
                        .plus(new Translation2d(offset * DashboardAlignment.BRANCH_OFFSET, 
                                                tagRotation.plus(Rotation2d.fromDegrees(90))));
                    double extraDistance = getTargetArmivatorState() == 2 ? Units.inchesToMeters(8) : 0;
                    Translation2d extrudedTranslation = branchTranslation
                        .plus(new Translation2d(distance + extraDistance, tagRotation));

                    // Add the extruded pose to the Pose2d List.
                    poses.add(new Pose2d(extrudedTranslation, tagRotation.plus(Rotation2d.fromDegrees(180))));
                }
            });
        }

        // Retrieve the requested pose and flip if on red alliance.
        Pose2d finalPose = poses.get(targetID - 1);
        return drivebase.isRedAlliance() ? FlippingUtil.flipFieldPose(finalPose) : finalPose;
    }

    /**
     * Calculates the closest pose to drive to based on the closest AprilTag on the reef.
     * @return The closest AprilTag Pose Offset.
     */
    public Pose2d calculateClosestReefPose() {
        return aprilTagFieldLayout.getTagPose(getClosestReefTag())
            .map(tagPose -> {
                Pose2d adjustedPose = tagPose.toPose2d().plus(new Transform2d(0, -0.0508, new Rotation2d()));

                // Extrude the target pose straight off the face of the tag.
                Rotation2d tagRotation = adjustedPose.getRotation();
                Translation2d extrudedTranslation = adjustedPose.getTranslation()
                    .plus(new Translation2d(DashboardAlignment.DISTANCE_AT_REEF, tagRotation));

                return new Pose2d(extrudedTranslation, tagRotation.plus(Rotation2d.fromDegrees(180)));
            })
            .orElse(new Pose2d());
    }

    /**
     * Retrieves the closest AprilTag on the reef to the current pose.
     * @return The closest AprilTag ID.
     */
    private int getClosestReefTag() {
        int closestAprilTagId = -1;
        double minDistance = Double.MAX_VALUE;
        boolean isRedAlliance = drivebase.isRedAlliance();

        // Determine which tags to search based on alliance.
        int startTag = isRedAlliance ? 6 : 17;
        int endTag = isRedAlliance ? 11 : 22;

        for (int id = startTag; id <= endTag; id++) {
            Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(id);
            if (tagPoseOptional.isPresent()) {
                Pose2d tagPose = tagPoseOptional.get().toPose2d();
                double distance = drivebase.getPose().getTranslation().getDistance(tagPose.getTranslation());

                if (distance < minDistance) {
                    minDistance = distance;
                    closestAprilTagId = id;
                }
            }
        }

        return closestAprilTagId;
    }

    /** 
     * Handles the state of the armivator based on the value from the dashboard.
     * @return The command to run based on the dashboard selected state.
     */
    public Command handleDashboardArmivatorState() {
        Map<Integer, Command> commandMap = Map.of(
            1, armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM),
            2, armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.L2),
            3, armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.TOP),
            4, armivatorCommands.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP)
        );

        return Commands.select(commandMap, this::getTargetArmivatorState);
    }

    /** Handles the state of the autonomous navigation system based on the value from the dashboard.
     * @return The command to run based on the dashboard selected state.
     */
    public Command handleDashboardPoseState() {
        return new SequentialCommandGroup(
            drivebase.driveToPose(
                calculateReefPose(
                    getTargetDashboardPose(),
                    DashboardAlignment.DISTANCE_AWAY_REEF
                ),
                AutonConstants.MAX_VELOCITY,
                AutonConstants.MAX_ACCELERATION
            ),
            drivebase.driveToPose(
                calculateReefPose(
                    getTargetDashboardPose(),
                    DashboardAlignment.DISTANCE_AT_REEF
                ),
                2.25,
                2.5
            )
        );
    }

    /**
     * Returns whether the closest tag ID to the robot pose contains a high algae.
     * @return Whether the closest tag ID contains a high algae.
     */
    public boolean isHighAlgae() {
        // Determine if the closest tag has a high algae.
        return (drivebase.isRedAlliance() && (getClosestReefTag() == 7 || getClosestReefTag() == 9 || getClosestReefTag() == 11)) ||
               (!drivebase.isRedAlliance() && (getClosestReefTag() == 18 || getClosestReefTag() == 20 || getClosestReefTag() == 22));
    }

    /**
     * Method to retrieve the selected pose via NetworkTables.
     * @return
     */
    private int getTargetDashboardPose() {
        return (int) SmartDashboard.getNumber("TargetDashboardPose", 1);
    }

    /** Method to retrieve the selected armivator state via NetworkTables. */
    private int getTargetArmivatorState() {
        return (int) SmartDashboard.getNumber("TargetArmivatorState", 1);
    }
}
