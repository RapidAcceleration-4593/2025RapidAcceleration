package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutonConstants.DashboardAlignment;
import frc.robot.commands.auton.utils.AutonUtils;

public class PoseNavigator extends SubsystemBase {

    /** AutonUtils Class Object. */
    private AutonUtils autonUtils;

    /** Notifier for Custom Dashboard. */
    private Notifier dashboardNotifier;

    /** Target Dashboard Pose, updated periodically through SmartDashboard. */
    private int targetDashboardPose;


    /**
     * Constructor for the PoseNavigator class.
     * Initializes the notifier that updates the SmartDashboard periodically.
     */
    public PoseNavigator(AutonUtils autonUtils) {
        this.autonUtils = autonUtils;

        dashboardNotifier = new Notifier(this::updateDashboard);
        dashboardNotifier.startPeriodic(0.2); // Run periodically, 200ms.
    }

    /**
     * Periodically updates the target pose and match time on the SmartDashboard.
     * This method is called by the Notifier.
     */
    private void updateDashboard() {
        targetDashboardPose = (int) SmartDashboard.getNumber("TargetDashboardPose", 1);

        SmartDashboard.putNumber("MatchTime", (int) DriverStation.getMatchTime());
        SmartDashboard.putBoolean("ManualControl", autonUtils.elevatorSubsystem.isHardManualControlEnabled());
    }

    /**
     * Selects the target pose based on the current dashboard state and alliance side.
     * @param distanceFromReef The distance from the robot's center to the reef, in meters.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @return The selected target {@link Pose2d} based on the current target dashboard pose.
     */
    public Pose2d selectTargetPose(double distanceFromReef, boolean isRedAlliance) {
        if (targetDashboardPose > 24) {
            return selectChutePose(targetDashboardPose);
        }
        return calculateReefPose(distanceFromReef, targetDashboardPose, isRedAlliance);
    }

    /**
     * Calculates the reef branch offsets and optionally returns a specific target pose.
     * @param distanceFromReef The distance to the reef.
     * @param targetID The target pose ID.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @return A {@link Pose2d} of the target ID.
     */
    public Pose2d calculateReefPose(double distanceFromReef, int targetID, boolean isRedAlliance) {
        List<Pose2d> poses = new ArrayList<>();

        // Loop through each side of the Reef.
        for (int sideIndex = 0; sideIndex < 6; sideIndex++) {
            // Midpoint angle for each side.
            double sideAngle = sideIndex * DashboardAlignment.ANGLE_INCREMENT + Math.PI;

            // Midpoint of the current Reef side.
            double midX = DashboardAlignment.REEF_RADIUS * Math.cos(sideAngle);
            double midY = DashboardAlignment.REEF_RADIUS * Math.sin(sideAngle);

            // Heading angle to face the center of the Reef.
            double headingAngle = Math.atan2(-midY, -midX);

            // Vector components along the side direction.
            double sideDirX = -Math.sin(sideAngle); // Perpendicular.
            double sideDirY = Math.cos(sideAngle);

            // Calculate branch positions.
            double[] branch1 = {midX - DashboardAlignment.BRANCH_OFFSET * sideDirX, midY - DashboardAlignment.BRANCH_OFFSET * sideDirY};
            double[] branch2 = {midX + DashboardAlignment.BRANCH_OFFSET * sideDirX, midY + DashboardAlignment.BRANCH_OFFSET * sideDirY};

            // Offset outward from branches to calculate poses.
            poses.add(createPose(branch1, sideAngle, distanceFromReef, headingAngle));
            poses.add(createPose(branch2, sideAngle, distanceFromReef, headingAngle));
        }

        // Convert target branch to zero-based index.
        int branchIndex = targetID - 1;

        // Target branch offset.
        Pose2d offsetPose = poses.get(branchIndex);

        // Transform the offset pose to the global field position.
        double transformedX = FieldConstants.REEF_POSE.getX() + offsetPose.getX();
        double transformedY = FieldConstants.REEF_POSE.getY() + offsetPose.getY();
        Rotation2d transformedHeading = offsetPose.getRotation();

        // Return final transformed pose, flipping if on red alliance.
        Pose2d finalPose = new Pose2d(new Translation2d(transformedX, transformedY), transformedHeading);
        return isRedAlliance ? FlippingUtil.flipFieldPose(finalPose) : finalPose;
    }

    /**
     * Selects a chute pose based on the target ID.
     * @param targetID The ID of the target chute.
     * @return The selected Pose2d corresponding to the chute.
     */
    private Pose2d selectChutePose(int targetID) {
        return switch (targetID) {
            case 25 -> autonUtils.RED_TOP_CHUTE[2];
            case 26 -> autonUtils.RED_TOP_CHUTE[1];
            case 27 -> autonUtils.RED_TOP_CHUTE[0];
            case 28 -> autonUtils.RED_BOTTOM_CHUTE[2];
            case 29 -> autonUtils.RED_BOTTOM_CHUTE[1];
            case 30 -> autonUtils.RED_BOTTOM_CHUTE[0];
            case 31 -> autonUtils.BLUE_TOP_CHUTE[2];
            case 32 -> autonUtils.BLUE_TOP_CHUTE[1];
            case 33 -> autonUtils.BLUE_TOP_CHUTE[0];
            case 34 -> autonUtils.BLUE_BOTTOM_CHUTE[2];
            case 35 -> autonUtils.BLUE_BOTTOM_CHUTE[1];
            case 36 -> autonUtils.BLUE_BOTTOM_CHUTE[0];
            default -> throw new IllegalArgumentException("Invalid chute: " + targetID);
        };
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
}
