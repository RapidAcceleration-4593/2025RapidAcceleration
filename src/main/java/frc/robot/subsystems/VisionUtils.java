package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.awt.Desktop;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget; 
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/** Utility class for handling vision-related functions like Object Detection and AprilTag localization. */
public class VisionUtils {

    /** AprilTag Field Layout of the year. */
    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    /** PhotonVision Simulation. */
    private VisionSystemSim visionSim;

    /** Current pose from the pose estimator using wheel odometry. */
    private Supplier<Pose2d> currentPose;

    /** Field from {@link swervelib.SwerveDrive#field} */
    private Field2d field2d;

    private PhotonCamera objectDetectionCamera;

    /**
     * Constructor for the VisionUtils class.
     * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
     * @param field Current field, should be {@link SwerveDrive#field}
     */
    public VisionUtils(Supplier<Pose2d> currentPose, Field2d field) {
        this.currentPose = currentPose;
        this.field2d = field;

        objectDetectionCamera = new PhotonCamera("Arducam_OV9782_Colored_2");

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(fieldLayout);

            for (Cameras camera : Cameras.values()) {
                camera.addToVisionSim(visionSim);
            }

            // openSimCameraViews();
        }
    }

    /**
     * Calculates a target pose relative to an AprilTag on the field.
     * @param aprilTag    The ID of the AprilTag.
     * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position.
     *                    itself correctly.
     * @return The target pose of the AprilTag.
     */
    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }
    }

    /**
     * Get the latest result from the camera.
     * @return The latest result from the camera, or an empty optional if no result is available.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
        List<PhotonPipelineResult> resultsList = objectDetectionCamera.getAllUnreadResults();
        return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the pose of an object detected by the camera.
     * @param currentPose The current pose of the robot.
     * @return The pose of the detected object, or an empty optional if no object was detected.
     */
    public Optional<Pose2d> getDetectedObjectPose(Pose2d currentPose) {
        Optional<PhotonPipelineResult> latestResult = getLatestResult();
    
        if (latestResult.isPresent() && latestResult.get().hasTargets()) {
            PhotonTrackedTarget target = latestResult.get().getBestTarget();
    
            double distanceToObject = PhotonUtils.calculateDistanceToTargetMeters(
                Units.inchesToMeters(30),
                Units.inchesToMeters(2.25),
                Units.degreesToRadians(-25),
                Units.degreesToRadians(target.getPitch())
            );

            double objectYaw = target.getYaw();
            double globalRotation = currentPose.getRotation().getDegrees() + 180 - objectYaw;

            double xTranslation = currentPose.getX() + (distanceToObject * Math.cos(Math.toRadians(globalRotation)));
            double yTranslation = currentPose.getY() + (distanceToObject * Math.sin(Math.toRadians(globalRotation)));

            Pose2d targetPose = new Pose2d(new Translation2d(xTranslation, yTranslation), Rotation2d.fromDegrees(globalRotation).plus(Rotation2d.kPi));

            System.out.println("Target Pose: " + targetPose);
            System.out.println("Target Distance: " + distanceToObject);
    
            return Optional.of(targetPose);
        }
    
        return Optional.empty();
    }
    
    /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
            /*
            * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
            * As a result, the odometry may not always be 100% accurate.
            * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
            * (This is why teams implement vision system to correct odometry.)
            * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
            */
            visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        }
        for (Cameras camera : Cameras.values()) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
            if (poseEst.isPresent()) {
                var pose = poseEst.get();
                swerveDrive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds,
                    camera.curStdDevs
                );
            }
        }
    }

    /**
     * Generates the estimated robot pose. Returns empty if:
     * <ul>
     *  <li> No Pose Estimates could be generated</li>
     * <li> The generated pose estimate was considered not accurate</li>
     * </ul>
     * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
        Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
        if (Robot.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            // Uncomment to enable outputting of vision targets in sim.
            poseEst.ifPresentOrElse(
                est ->
                    debugField
                        .getObject("VisionEstimation")
                        .setPose(est.estimatedPose.toPose2d()),
                () -> {
                    debugField.getObject("VisionEstimation").setPoses();
                });
        }
        return poseEst;
    }

    /**
     * Get distance of the robot from the AprilTag pose.
     * @param id AprilTag ID.
     * @return Distance.
     */
    public double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

    /**
     * Get tracked target from a camera of AprilTagID.
     * @param id AprilTag ID.
     * @param camera Camera to check.
     * @return Tracked target.
     */
    public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
        PhotonTrackedTarget target = null;
        for (PhotonPipelineResult result : camera.resultsList) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget i : result.getTargets()) {
                    if (i.getFiducialId() == id) {
                        return i;
                    }
                }
            }
        }
        return target;
    }

    /**
     * Vision simulation.
     * @return Vision Simulation.
     */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    /** Open up the PhotonVision camera streams on the localhost, assumes running PhotonVision on localhost. */
    @SuppressWarnings("unused")
    private void openSimCameraViews() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try {
                Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
                Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
                Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /** Update the {@link Field2d} to include tracked targets. */
    public void updateVisionField() {
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for (Cameras camera : Cameras.values()) {
            if (!camera.resultsList.isEmpty()) {
                PhotonPipelineResult latest = camera.resultsList.get(0);
                if (latest.hasTargets()) {
                    targets.addAll(latest.targets);
                }
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
                poses.add(targetPose);
            }
        }

        field2d.getObject("tracked targets").setPoses(poses);
    }

    /** Camera Enum to select each camera. */
    enum Cameras {
        OV9782_Colored_1("Arducam_OV9281_Monochrome",
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(0)),
                new Translation3d(Units.inchesToMeters(10),
                                  Units.inchesToMeters(-9),
                                  Units.inchesToMeters(19.5)),
                VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),

        OV9782_Colored_2("Arducam_OV9782_Colored_1",
                new Rotation3d(0, Units.degreesToRadians(15), 0),
                new Translation3d(Units.inchesToMeters(10),
                                  Units.inchesToMeters(9),
                                  Units.inchesToMeters(19.5)),
                VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

        /** Latency alert to use when high latency is detected. */
        public final Alert latencyAlert;

        /** Camera instance for comms. */
        public final PhotonCamera camera;
        
        /**  Pose estimator for camera. */
        public final PhotonPoseEstimator poseEstimator;

        /** Standard Deviation for single tag readings for pose estimation. */
        private final Matrix<N3, N1> singleTagStdDevs;

        /** Standard deviation for multi-tag readings for pose estimation. */
        private final Matrix<N3, N1> multiTagStdDevs;
        
        /** Transform of the camera rotation and translation relative to the center of the robot. */
        private final Transform3d robotToCamTransform;

        /** Current standard deviations used. */
        public Matrix<N3, N1> curStdDevs;

        /** Estimated robot pose. */
        public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

        /** Simulated camera instance which only exists during simulations. */
        public PhotonCameraSim cameraSim;

        /** Results list to be updated periodically and cached to avoid unnecessary queries. */
        public List<PhotonPipelineResult> resultsList = new ArrayList<>();
        
        /**
         * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
         * estimation noise on an actual robot.
         * @param name Name of the PhotonVision camera found in the PV UI.
         * @param robotToCamRotation {@link Rotation3d} of the camera.
         * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
         * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the camera.
         * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
         */
        Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
            camera = new PhotonCamera(name);

            // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

            poseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                    robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevsMatrix;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                // A 640 x 480 camera with a 70 degree diagonal FOV.
                cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
                // Approximate detection noise with average and standard deviation error in pixels.
                cameraProp.setCalibError(0.25, 0.08);
                // Set the camera image capture framerate (Note: this is limited by robot loop rate).
                cameraProp.setFPS(30);
                // The average and standard deviation in milliseconds of image data latency.
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);

                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }

        /**
         * Add camera to {@link VisionSystemSim} for simulated photon vision.
         * @param systemSim {@link VisionSystemSim} to use.
         */
        public void addToVisionSim(VisionSystemSim systemSim) {
            if (Robot.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform);
            }
        }

        /**
         * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
         * recent result!
         * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
         */
        public Optional<PhotonPipelineResult> getBestResult() {
            if (resultsList.isEmpty()) {
                return Optional.empty();
            }

            PhotonPipelineResult bestResult = resultsList.get(0);
            double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
            double currentAmbiguity = 0;
            for (PhotonPipelineResult result : resultsList) {
                currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
                if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
                    bestResult = result;
                    amiguity = currentAmbiguity;
                }
            }
            return Optional.of(bestResult);
        }

        /**
         * Get the latest result from the current cache.
         * @return Empty optional if nothing is found. Latest result if something is there.
         */
        public Optional<PhotonPipelineResult> getLatestResult() {
            return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
        }

        /**
         * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
         * cache of results.
         * @return Estimated pose.
         */
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }

        /** Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp. */
        private void updateUnreadResults() {
            double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
            for (PhotonPipelineResult result : resultsList) {
                mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
            }
            resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
            resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
            });
            if (!resultsList.isEmpty()) {
                updateEstimatedGlobalPose();
            }
        }

        /**
         * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
         * per loop.
         * 
         * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
         * {@link Cameras#updateEstimationStdDevs}
         *
         * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for estimation.
         */
        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var change : resultsList) {
                visionEst = poseEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
            estimatedRobotPose = visionEst;
        }

        /**
         * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
         * on number of tags, estimation strategy, and distance from the tags.
         * @param estimatedPose The estimated pose to guess standard deviations for.
         * @param targets       All targets in this camera frame
         */
        private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose.isEmpty()) {
                // No pose input. Default to single-tag std devs
                curStdDevs = singleTagStdDevs;
            } else {
                // Pose present. Start running Heuristic
                var estStdDevs = singleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (var tgt : targets) {
                    var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) {
                        continue;
                    }
                    numTags++;
                    avgDist +=
                        tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags == 0) {
                    // No tags visible. Default to single-tag std devs
                    curStdDevs = singleTagStdDevs;
                } else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) {
                        estStdDevs = multiTagStdDevs;
                    }
                    // Increase std devs based on (average) distance
                    if (numTags == 1 && avgDist > 4) {
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    } else {
                        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                    }
                    curStdDevs = estStdDevs;
                }
            }
        }
    }
}