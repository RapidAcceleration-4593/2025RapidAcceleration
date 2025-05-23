// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.utils.VisionUtils;

import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    /** Swerve Drive Object. */
    private final SwerveDrive swerveDrive;

    /** PhotonVision class to keep an accurate odometry. */
    private VisionUtils visionUtils;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                        new Pose2d(new Translation2d(Meter.of(2),
                                                                                                    Meter.of(4)),
                                                                                Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new IllegalStateException(e);
        }

        // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setHeadingCorrection(false);

        // Disable cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setCosineCompensator(false);

        // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setAngularVelocityCompensation(false,
                                                     false,
                                          0.1);

        // Resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                    1);

        if (AutonConstants.DRIVE_WITH_VISION) {
            setupPhotonVision();
            
            // Stop the odometry thread if we are using vision that way we can synchronize updates better.
            swerveDrive.stopOdometryThread();
        }
        setupPathPlanner();
    }

    /**
     * Construct the swerve drive.
     * @param driveCfg SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg,
                                      controllerCfg,
                                      Constants.MAX_SPEED,
                                      new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                                 Rotation2d.fromDegrees(0)));
    }

    /** Setup the photon vision class. */
    public void setupPhotonVision() {
        this.visionUtils = new VisionUtils(swerveDrive::getPose, swerveDrive.field);
    }

    public VisionUtils getVisionUtils() {
        return this.visionUtils;
    }

    @Override
    public void periodic() {
        if (AutonConstants.DRIVE_WITH_VISION) {
            swerveDrive.updateOdometry();
            visionUtils.updatePoseEstimation(swerveDrive);
        }
    }

    /** Setup AutoBuilder for PathPlanner. */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. Store this in Constants file.
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last.
            AutoBuilder.configure(
                this::getPose,
                // Robot pose supplier.
                this::resetOdometry,
                // Method to reset odometry (will be called if your auto has a starting pose).
                this::getRobotVelocity,
                // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE.
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedforward) {
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces());
                    } else {
                        setChassisSpeeds(speedsRobotRelative);
                    }
                },
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards.
                new PPHolonomicDriveController(
                    // PPHolonomicController is the built in path following controller for holonomic drive trains.
                    new PIDConstants(5.0, 0.0, 0.0),
                    // Translation PID constants.
                    new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants.
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE.
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                }, this);

        } catch (Exception e) {
            // Handle exception, as needed.
            e.printStackTrace();
        }

        // Preload PathPlanner PathFinding.
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     * @param pose Target {@link Pose2d} to go to.
     * @param maxVelocity Maximum velocity to use.
     * @param maxAcceleration Maximum acceleration to use.
     * @return PathFinding command.
     */
    public Command driveToPose(Pose2d pose, double maxVelocity, double maxAcceleration) {
        // Create the constraints to use while pathfinding.
        PathConstraints constraints = new PathConstraints(
            maxVelocity, maxAcceleration,
            swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720)
        );

        // Since AutoBuilder is configured, we can use it to build pathfinding commands.
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            0
        );
    }

    /**
     * Drive the robot given a chassis field oriented velocity. 
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    /**
     * Get the swerve drive kinematics object. 
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     * @param initialHolonomicPose The pose to set the odometry to.
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
    
    /**
     * Retrieve the maximum angular chassis velocity.
     * @return The maximum angular velocity of the chassis.
     */
    public double getMaximumChassisAngularVelocity() {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

    /**
     * Post the trajectory to the field.
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /** Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0. */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     * @return true if the red alliance, false if blue. Defaults to false if none is available.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward.
     * <p>If red alliance rotate the robot 180 after the drivebase zero command.
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180))); // Set the pose 180 degrees.
        } else {
            zeroGyro();
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot.
     * @return A ChassisSpeeds object of the current field-relative velocity.
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot.
     * @return A {@link ChassisSpeeds} object of the current velocity.
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /** Lock the swerve drive to prevent it from moving. */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the IMU.
     * @return The heading as a {@link Rotation2d} angle.
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Gets the swerve drive object.
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}