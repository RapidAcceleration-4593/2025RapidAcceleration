package frc.robot.subsystems.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants.DashboardAlignment;
import frc.robot.commands.armivator.ArmivatorCommands;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseNavigator extends SubsystemBase {

    /** SwerveSubsystem Class Object. */
     private final SwerveSubsystem drivebase;

    /** ArmivatorCommands Object. */
    public final ArmivatorCommands armivatorCommands;

    /** Notifier for Custom Dashboard. */
    private final Notifier dashboardNotifier;

    /** AprilTag field layout. */
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    /**
     * Constructor for the PoseNavigator class.
     * Initializes the notifier that updates the SmartDashboard periodically.
     */
    public PoseNavigator(SwerveSubsystem drivebase, ArmivatorCommands armivatorCommands) {
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
     * Returns whether the closest tag ID to the robot pose contains a high algae.
     * @return Whether the closest tag ID contains a high algae.
     */
    public boolean isHighAlgae() {
        return (drivebase.isRedAlliance() && Arrays.asList(7, 9, 11).contains(getClosestReefTag())) ||
               (!drivebase.isRedAlliance() && Arrays.asList(18, 20, 22).contains(getClosestReefTag()));
    }

    /** Method to retrieve the selected pose via NetworkTables. */
    public int getTargetDashboardPose() {
        return (int) SmartDashboard.getNumber("TargetDashboardPose", 1);
    }

    /** Method to retrieve the selected armivator state via NetworkTables. */
    public int getTargetArmivatorState() {
        return (int) SmartDashboard.getNumber("TargetArmivatorState", 1);
    }
}
