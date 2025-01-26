package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class PoseNavigator extends SubsystemBase {

    /** Notifier for Custom Dashboard. */
    private Notifier dashboardNotifier;

    /** Target Dashboard Pose, updated periodically through SmartDashboard. */
    private int targetDashboardPose;

    /** Match Time reflected by FMS. */
    private int lastMatchTime = -1;

    /**
     * Constructor for the PoseNavigator class.
     * Initializes the notifier that updates the SmartDashboard periodically.
     */
    public PoseNavigator() {
        dashboardNotifier = new Notifier(this::updateDashboard);
        dashboardNotifier.startPeriodic(0.2); // Run every 200ms
    }

    /**
     * Periodically updates the target pose and match time on the SmartDashboard.
     * This method is called by the Notifier.
     */
    private void updateDashboard() {
        int newTargetPose = (int) SmartDashboard.getNumber("TargetDashboardPose", 0);
        if (newTargetPose != targetDashboardPose) {
            targetDashboardPose = newTargetPose;
        }

        int currentMatchTime = (int) DriverStation.getMatchTime();
        if (currentMatchTime != lastMatchTime) {
            SmartDashboard.putNumber("MatchTime", currentMatchTime);
            lastMatchTime = currentMatchTime;
        }

        // Flushes all updated values.
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Selects the target pose based on the current dashboard state and alliance side.
     * @param distanceFromReef The distance from the robot's center to the reef, in meters.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @return The selected target {@link Pose2d} based on the current target dashboard pose.
     */
    public Pose2d selectTargetPose(double distanceFromReef, boolean isRedAlliance) {
        return getPoseFromDashboardState(targetDashboardPose, distanceFromReef, isRedAlliance);
    }

    /**
     * Calculates the robot pose offsets of each branch around the reef.
     * <p>
     * This method generates a list of 12 poses around the reef (2 per side),
     * calculating the heading to face the center of the reef for each pose.
     * @param distanceFromReef The distance from the robot's center to the reef, in meters.
     * @return A list of {@link Pose2d} objects representing the robot's position and heading.
     * @throws IllegalArgumentException If the distance is outside the valid range (0.4 to 1.5 meters).
     */
    public List<Pose2d> calculateReefBranchOffsets(double distanceFromReef) {
        // Validate distance is within the valid range, in meters.
        if (distanceFromReef < 0.4 || distanceFromReef > 1.5) {
            throw new IllegalArgumentException("Distance must be between 0.4 and 1.5 meters. Provided: " + distanceFromReef);
        }

        List<Pose2d> poses = new ArrayList<>();

        // Reef Parameters.
        final double radius = Units.inchesToMeters(32.75);
        final double branchOffset = Units.inchesToMeters(6.5);
        final double angleIncrement = Math.toRadians(60.0);

        // Loop through each side of the Reef.
        for (int i = 0; i < 6; i++) {
            // Midpoint angle for each side.
            double sideAngle = i * angleIncrement;

            // Midpoint of the current Reef side.
            double midX = radius * Math.cos(sideAngle);
            double midY = radius * Math.sin(sideAngle);

            // Heading angle to face the center of the Reef.
            double headingAngle = Math.atan2(-midY, -midX);

            // Vector components along the side direction.
            double sideDirX = -Math.sin(sideAngle); // Perpendicular.
            double sideDirY = Math.cos(sideAngle);

            double[] branch1 = {midX - branchOffset * sideDirX, midY - branchOffset * sideDirY};
            double[] branch2 = {midX + branchOffset * sideDirX, midY + branchOffset * sideDirY};

            // Offset outward from branches to calculate poses.
            poses.add(createPose(branch1, sideAngle, distanceFromReef, headingAngle));
            poses.add(createPose(branch2, sideAngle, distanceFromReef, headingAngle));
        }

        return poses;
    }

    /**
     * Creates a {@link Pose2d} object given branch position, angle, distance, and heading.
     * @param branch The branch position (x, y).
     * @param sideAngle The angle of the side of the reef.
     * @param distanceFromReef The distance from the robot to the reef, in meters.
     * @param headingAngle The heading angle to face the reef center.
     * @return The created {@link Pose2d} object.
     */
    private Pose2d createPose(double[] branch, double sideAngle, double distanceFromReef, double headingAngle) {
        double transformedX = branch[0] + distanceFromReef * Math.cos(sideAngle);
        double transformedY = branch[1] + distanceFromReef * Math.sin(sideAngle);
        return new Pose2d(new Translation2d(transformedX, transformedY), new Rotation2d(headingAngle));
    }

    /**
     * Selects the target pose based on the target ID and alliance side.
     * @param targetID The ID of the target pose.
     * @param distanceFromReef The distance from the robot to the reef, in meters.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @return The target pose corresponding to the target ID.
     */
    public Pose2d getPoseFromDashboardState(int targetID, double distanceFromReef, boolean isRedAlliance) {
        if (targetID >= 13 && targetID <= 36) {
            return selectChutePose(targetID);
        }
        return calculateReefPose(distanceFromReef, targetID, isRedAlliance);
    }

    /**
     * Selects a chute pose based on the target ID.
     * @param targetID The ID of the target chute.
     * @return The selected Pose2d corresponding to the chute.
     */
    private Pose2d selectChutePose(int targetID) {
        return switch (targetID) {
            case 13 -> FieldConstants.BOTTOM_RED_CHUTE_RIGHT;
            case 14 -> FieldConstants.BOTTOM_RED_CHUTE_MIDDLE;
            case 15 -> FieldConstants.BOTTOM_RED_CHUTE_LEFT;
            case 16 -> FieldConstants.TOP_RED_CHUTE_RIGHT;
            case 17 -> FieldConstants.TOP_RED_CHUTE_MIDDLE;
            case 18 -> FieldConstants.TOP_RED_CHUTE_LEFT;
            case 31 -> FieldConstants.TOP_BLUE_CHUTE_RIGHT;
            case 32 -> FieldConstants.TOP_BLUE_CHUTE_MIDDLE;
            case 33 -> FieldConstants.TOP_BLUE_CHUTE_LEFT;
            case 34 -> FieldConstants.BOTTOM_BLUE_CHUTE_RIGHT;
            case 35 -> FieldConstants.BOTTOM_BLUE_CHUTE_MIDDLE;
            case 36 -> FieldConstants.BOTTOM_BLUE_CHUTE_LEFT;
            default -> throw new IllegalArgumentException("Invalid chute: " + targetID);
        };
    }

    /**
     * Calculates the pose of a specific branch at the reef with a given offset.
     * @param distanceFromReef The distance to the reef.
     * @param targetID The branch ID.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @return The calculated Pose2d for the target branch.
     */
    private Pose2d calculateReefPose(double distanceFromReef, int targetID, boolean isRedAlliance) {
        // Convert target branch to zero-based index.
        int branchIndex = targetID - 1;

        // Get base reef position for specified alliance.
        double[] basePose = isRedAlliance ? FieldConstants.RED_REEF_POSE : FieldConstants.BLUE_REEF_POSE;

        // Offsets for all reef branches.
        List<Pose2d> reefBranchOffsets = calculateReefBranchOffsets(distanceFromReef);

        // Target branch offset.
        Pose2d offsetPose = reefBranchOffsets.get(branchIndex);

        // Transform the offset pose to the global field position.
        double transformedX = basePose[0] + offsetPose.getX();
        double transformedY = basePose[1] + offsetPose.getY();
        Rotation2d transformedHeading = offsetPose.getRotation();

        // Transformed pose after calculations.
        return new Pose2d(new Translation2d(transformedX, transformedY), transformedHeading);
    }
}
