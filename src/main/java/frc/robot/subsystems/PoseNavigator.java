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

    /** TargetDashboardPose reflected by SmartDashboard. */
    private int targetDashboardPose;

    /** Match Time reflected by FMS. */
    private int lastMatchTime = -1;

    /**
     * Constructor for the PoseNavigator class.
     * Initializes the notifier that updates the SmartDashboard periodically.
     */
    public PoseNavigator() {
        dashboardNotifier = new Notifier(() -> {
            // Update SmartDashboard periodically, separate from Command Scheduler.
            int newTargetReefBranch = (int) SmartDashboard.getNumber("TargetDashboardPose", 0);
            if (newTargetReefBranch != targetDashboardPose) {
             targetDashboardPose = newTargetReefBranch;
            }
 
            int currentMatchTime = (int) DriverStation.getMatchTime();
            if (currentMatchTime != lastMatchTime) {
                SmartDashboard.putNumber("MatchTime", currentMatchTime);
                lastMatchTime = currentMatchTime;
            }
 
            // Flushes all updated values. 
            NetworkTableInstance.getDefault().flush();
        });
 
        // Start the Notifier to run every 0.2 seconds (200ms).
        dashboardNotifier.startPeriodic(0.2);
    }

    public Pose2d foo() {
        return selectTargetDashboardPose(targetDashboardPose, isRedAlliance());
    }

    /**
     * Retrieves the robot's pose offsets for branches around the reef.
     * <p>
     * The method generates 12 poses around the reef (2 per side), calculating
     * the heading to face the center of the reef for each pose.
     * @param distanceToReef The distance from the robot's center to the reef, in meters.
     * @return A list of {@link Pose2d} objects representing the robot's position and heading.
     * @throws IllegalArgumentException If the distance is outside the valid range (0.4 to 1.5 meters).
     */
    public List<Pose2d> calculateReefBranchOffsets(double distanceToReef) {
        if (distanceToReef < 0.4 || distanceToReef > 1.5) {
            throw new IllegalArgumentException("Distance must be between 0.4 and 1.5 meters. Provided: " + distanceToReef);
        }

        List<Pose2d> poses = new ArrayList<>();
        final double radius = Units.inchesToMeters(32.75);
        final double branchOffset = Units.inchesToMeters(6.5);
        final double angleIncrement = Math.toRadians(60.0);

        // Loop through each side of the hexagon.
        for (int i = 0; i < 6; i++) {
            // Midpoint angle for each side.
            double sideAngle = i * angleIncrement;

            // Midpoint of the current hexagon side.
            double midX = radius * Math.cos(sideAngle);
            double midY = radius * Math.sin(sideAngle);

            // Heading angle to face the center of the hexagon.
            double headingAngle = Math.atan2(-midY, -midX);

            // Vector components along the side direction.
            double sideDirX = -Math.sin(sideAngle); // Perpendicular to the normal side.
            double sideDirY = Math.cos(sideAngle);

            double[] branch1 = {midX - branchOffset * sideDirX, midY - branchOffset * sideDirY};
            double[] branch2 = {midX + branchOffset * sideDirX, midY + branchOffset * sideDirY};

            // Offset outward from branches to calculate poses.
            poses.add(createPose(branch1, sideAngle, distanceToReef, headingAngle));
            poses.add(createPose(branch2, sideAngle, distanceToReef, headingAngle));
        }

        return poses;
    }

    /**
     * Creates a {@link Pose2d} object given branch position, angle, distance, and heading.
     * @param branch The branch position (x, y).
     * @param sideAngle The angle of the side of the reef.
     * @param distanceToReef The distance from the robot's center to the reef.
     * @param headingAngle The heading angle to face the reef center.
     * @return The created {@link Pose2d} object.
     */
    private Pose2d createPose(double[] branch, double sideAngle, double distanceToReef, double headingAngle) {
        double transformedX = branch[0] + distanceToReef * Math.cos(sideAngle);
        double transformedY = branch[1] + distanceToReef * Math.sin(sideAngle);
        return new Pose2d(new Translation2d(transformedX, transformedY), new Rotation2d(headingAngle));
    }

    /**
     * Selects the target pose based on the target ID and alliance side.
     * @param targetID The ID of the target pose.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @return The target pose corresponding to the target ID.
     */
    public Pose2d getPoseFromDashboardState(int targetID, double distanceToReef, boolean isRedAlliance) {
        if (targetID >= 13 && targetID <=36) {
            return selectChutePose(targetID);
        }
        return calculateReefPose(distanceToReef, targetID, isRedAlliance);
    }

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
        return findReefBranchPose(0.5, targetID, isRedAlliance);
    }

    private Pose2d findReefBranchPose(double distanceToReef, int targetID, boolean isRedAlliance) {
        // Convert targetBranch to zero-based index.
        int branchIndex = targetID - 1;

        // Get the base reef position for the specified alliance.
        double[] basePose = isRedAlliance
            ? FieldConstants.RED_REEF_POSE
            : FieldConstants.BLUE_REEF_POSE;

        // Get the offsets for all reef branches.
        List<Pose2d> reefBranchOffsets = calculateReefBranchOffsets(distanceToReef);
        Pose2d offsetPose = reefBranchOffsets.get(branchIndex);

        // Transform the offset pose to the global field position.
        double transformedX = basePose[0] + offsetPose.getX();
        double transformedY = basePose[1] + offsetPose.getY();
        Rotation2d transformedHeading = offsetPose.getRotation();

        // Transformed pose from calculations above.
        return new Pose2d(new Translation2d(transformedX, transformedY), transformedHeading);
    }

    /**
     * Calculate the robot's Pose2d based on the alliance color, a specified side, and
     * a given distance from the center of the reef.
     * @param robotPose                 The current global pose of the robot.
     * @param isRedAlliance             Indicates if the robot is on the red alliance.
     * @param distanceOffset            Distance between the center of the reef and desired
     *                                  robot pose, between 1.2 and 3.0, in meters.
     * @return                          A {@link Pose2d} object representing the calculated
     *                                  closest position and orientation of a reef.
     * @throws IllegalArgumentException If the provided side is not between 1 and 6, or
     *                                  if the distance is not between 1.2 and 3.0 (inclusive).
     */
    public Pose2d findClosestReefSidePose(Pose2d robotPose, boolean isRedAlliance, double distanceOffset) {
        // Validate distance is within the valid range, in meters.
        if (distanceOffset < 1.2 || distanceOffset > 3.0) {
            throw new IllegalArgumentException("Distance must be between 1.2 and 3.0. Provided: " + distanceOffset);
        }

        double minDistance = Double.MAX_VALUE; // Initial, large number.
        Pose2d closestPose = null;

        // Iterate through all sides (1 through 6) of the reef.
        for (int side = 1; side <= 6; side++) {
            double rotationDegrees = 60 * (side - 1); // Calculate rotation in degrees based on the side selected.
            double rotationRadians = Math.toRadians(rotationDegrees + 180); // Convert rotation, plus 180, to radians for calculations.

            // Calculate horizontal and vertical offsets using trigonometry.
            double horizontalOffset = distanceOffset * Math.cos(rotationRadians);
            double verticalOffset = distanceOffset * Math.sin(rotationRadians);

            // Select the appropriate reef position based on alliance color.
            double[] allianceReef = isRedAlliance ? FieldConstants.RED_REEF_POSE : FieldConstants.BLUE_REEF_POSE;

            // Calculate the final position with offsets.
            double xPosition = allianceReef[0] + horizontalOffset;
            double yPosition = allianceReef[1] + verticalOffset;

            // Calculate pose for the current side.
            Pose2d sidePose = new Pose2d(xPosition, yPosition, Rotation2d.fromDegrees(rotationDegrees));

            // Calculate distance from the robot's pose to the side's pose.
            double distance = calculatePoseDistance(robotPose, sidePose);

            // Check if current pose is closer than previous closest.
            if (distance < minDistance) {
                minDistance = distance;
                closestPose = sidePose;
            }
        }

        return closestPose;
    }

    /**
     * Calculates the Euclidean distance between two poses.
     * @param pose1 The first pose.
     * @param pose2 The second pose.
     * @return      The distance between the two poses.
     */
    private double calculatePoseDistance(Pose2d pose1, Pose2d pose2) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();

        return Math.hypot(dx, dy);
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     * @return true if the red alliance, false if blue. Defaults to false if none is available.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
}
