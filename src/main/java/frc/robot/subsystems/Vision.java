package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Vision extends SubsystemBase {
    private static final String DEBUG_OUTPUT_ENABLED_KEY = "Vision/DebugEnabled";
    private static final String DYNAMICS_FILTER_ENABLED_KEY = "Vision/Filter/RobotDynamicsEnabled";
    private static final String MAX_ACCEPTABLE_VELOCITY_MPS_KEY = "Vision/Filter/MaxRobotVelocityMps";
    private static final String MAX_ACCEPTABLE_ACCEL_MPS2_KEY = "Vision/Filter/MaxRobotAccelerationMpsSq";
    private static final String MAX_ACCEPTABLE_ANGULAR_ACCEL_RADPS2_KEY = "Vision/Filter/MaxRobotAngularAccelerationRadPerSecSq";
    private static final double DEFAULT_MAX_ACCEPTABLE_VELOCITY_MPS = 2.5;
    private static final double DEFAULT_MAX_ACCEPTABLE_ACCEL_MPS2 = 3.0;
    private static final double DEFAULT_MAX_ACCEPTABLE_ANGULAR_ACCEL_RADPS2 = 8.0;

    private record CameraPoseSource(
            String label,
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            Transform3d robotToCamera,
            boolean useForHubObservation) {
    }

    private record MeasurementStats(
            int tagCount,
            double closestTagDistanceMeters,
            double averageTagDistanceMeters,
            double farthestTagDistanceMeters,
            double maxAmbiguity) {
    }

    private record MotionStats(
            double robotVelocityMetersPerSec,
            double robotAccelerationMetersPerSecSq,
            double robotAngularAccelerationRadPerSecSq) {
    }

    public record HubObservation(
            Translation2d robotToHubFrontMeters,
            double rangeToHubCenterMeters,
            int targetCount,
            String source) {
    }

    public record LocalHubAimObservation(
            Translation2d robotToGoalMeters,
            double rangeToGoalMeters,
            int fiducialId,
            String source) {
    }

    private final Drivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout;
    private final List<CameraPoseSource> cameraPoseSources;
    private final Map<String, Optional<PhotonPipelineResult>> latestResults = new HashMap<>();
    private final Map<String, Double> lastAcceptedTimestamps = new HashMap<>();
    private final Map<String, Integer> fuseCounts = new HashMap<>();
    private boolean poseSeededFromVision = false;
    private boolean debugOutputEnabled = false;

    // Vision fusion tuning constants (hardcoded; not configurable via SmartDashboard)
    private static final double MAX_TRANSLATION_JUMP_M = 3.0;
    private static final double MAX_ROTATION_JUMP_DEG = 90.0;
    private static final boolean ENABLE_POSE_FUSION = true;
    private static final boolean ENABLE_INITIAL_POSE_SEED = true;
    private static final double MAX_MEASUREMENT_AGE_SEC = 0.5;
    private static final double FIELD_BOUNDS_MARGIN_M = 0.3;
    private static final double BUMP_HARD_RESET_TRANSLATION_ERROR_M = 1.0;
    private static final int BUMP_HARD_RESET_TAG_COUNT = 2;
    private static final double MAX_TAG_DISTANCE_M = 4.5;
    private static final double MAX_SINGLE_TAG_DISTANCE_M = 4.0;
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.35;
    private static final double MAX_MOVING_TAG_DISTANCE_M = 3.2;
    private static final double MAX_MOVING_AVERAGE_TAG_DISTANCE_M = 2.8;
    private static final double MAX_MOVING_SINGLE_TAG_DISTANCE_M = 2.2;
    private static final double MAX_MOVING_SINGLE_TAG_AMBIGUITY = 0.15;
    private static final double STATIONARY_POSE_RECOVERY_TRANSLATION_ERROR_M = 3.0;
    private static final double STATIONARY_POSE_RECOVERY_CLOSE_TAG_DISTANCE_M = 2.5;
    private static final String VISION_POSE_UPDATES_ALLOWED_KEY = "Vision/PoseUpdatesAllowedByRobotState";
    private static final Set<Integer> IGNORED_TAG_IDS = Set.of(7, 23);

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        this.fieldLayout = layout;

        cameraPoseSources = List.of(
                createCameraSource(
                        "Left",
                        Constants.Vision.LEFT_CAMERA_NAME,
                        Constants.Vision.ROBOT_TO_LEFT_CAMERA,
                        layout,
                        true),
                createCameraSource(
                        "Right",
                        Constants.Vision.RIGHT_CAMERA_NAME,
                        Constants.Vision.ROBOT_TO_RIGHT_CAMERA,
                        layout,
                        true),
                createCameraSource(
                        "SideLeft",
                        Constants.Vision.SIDE_LEFT_CAMERA_NAME,
                        Constants.Vision.ROBOT_TO_SIDE_LEFT_CAMERA,
                        layout,
                        false),
                createCameraSource(
                        "SideRight",
                        Constants.Vision.SIDE_RIGHT_CAMERA_NAME,
                        Constants.Vision.ROBOT_TO_SIDE_RIGHT_CAMERA,
                        layout,
                        false));

        SmartDashboard.putBoolean(DEBUG_OUTPUT_ENABLED_KEY, false);
        putDebugBoolean("Vision/LastResetUsedVision", false);
        SmartDashboard.putBoolean(DYNAMICS_FILTER_ENABLED_KEY, true);
        SmartDashboard.putNumber(MAX_ACCEPTABLE_VELOCITY_MPS_KEY, DEFAULT_MAX_ACCEPTABLE_VELOCITY_MPS);
        SmartDashboard.putNumber(MAX_ACCEPTABLE_ACCEL_MPS2_KEY, DEFAULT_MAX_ACCEPTABLE_ACCEL_MPS2);
        SmartDashboard.putNumber(
                MAX_ACCEPTABLE_ANGULAR_ACCEL_RADPS2_KEY,
                DEFAULT_MAX_ACCEPTABLE_ANGULAR_ACCEL_RADPS2);
        putDebugBoolean(VISION_POSE_UPDATES_ALLOWED_KEY, false);
    }

    private CameraPoseSource createCameraSource(
            String label,
            String cameraName,
            Transform3d robotToCamera,
            AprilTagFieldLayout layout,
            boolean useForHubObservation) {
        PhotonCamera camera = new PhotonCamera(cameraName);
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        return new CameraPoseSource(label, camera, estimator, robotToCamera, useForHubObservation);
    }

    @Override
    public void periodic() {
        debugOutputEnabled = SmartDashboard.getBoolean(DEBUG_OUTPUT_ENABLED_KEY, false);

        for (CameraPoseSource source : cameraPoseSources) {
            processCamera(source);
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose3d hubCenter = FieldConstants.Hub.getCenterForAllianceOrNearest(alliance, drivetrain.getState().Pose);
        double poseDistanceMeters = drivetrain.getState().Pose.getTranslation()
                .getDistance(hubCenter.toPose2d().getTranslation());
        Optional<HubObservation> hubObservation = getHubObservation(alliance);

        putDebugString("Vision/Alliance", alliance.map(Enum::name).orElse("Unknown"));
        putDebugNumber("Vision/TargetDistanceMeters", poseDistanceMeters);
        SmartDashboard.putNumber("Vision/TargetDistanceInches", Units.metersToInches(poseDistanceMeters));
        putDebugBoolean("Vision/HasObservedTargetDistance", hubObservation.isPresent());
        putDebugNumber(
                "Vision/ObservedTargetDistanceMeters",
                hubObservation.map(HubObservation::rangeToHubCenterMeters).orElse(-1.0));
        putDebugNumber(
                "Vision/ObservedTargetDistanceInches",
                hubObservation.map(obs -> Units.metersToInches(obs.rangeToHubCenterMeters())).orElse(-1.0));
    }

    public Optional<HubObservation> getHubObservation(Optional<Alliance> alliance) {
        List<Translation2d> robotToTargets = new ArrayList<>();

        for (CameraPoseSource source : cameraPoseSources) {
            if (!source.useForHubObservation()) {
                continue;
            }

            getLatestStoredResult(source.label()).ifPresent(result ->
                    collectHubTargets(result, source.robotToCamera(), alliance, robotToTargets));
        }

        if (robotToTargets.isEmpty()) {
            return Optional.empty();
        }

        Translation2d average = new Translation2d();
        for (Translation2d target : robotToTargets) {
            average = average.plus(target);
        }
        average = average.div(robotToTargets.size());

        double rangeToHubCenterMeters = average.getNorm() + FieldConstants.Hub.FRONT_FACE_TO_CENTER_XY_METERS;
        return Optional.of(new HubObservation(average, rangeToHubCenterMeters, robotToTargets.size(), "tags"));
    }

    public Optional<LocalHubAimObservation> getLocalHubAimObservation(Optional<Alliance> alliance) {
        int preferredTagId = getPreferredHubFrontTagId(alliance);
        if (preferredTagId <= 0) {
            return Optional.empty();
        }

        for (CameraPoseSource source : cameraPoseSources) {
            if (!source.useForHubObservation()) {
                continue;
            }

            Optional<LocalHubAimObservation> observation = getLatestStoredResult(source.label())
                    .flatMap(result -> extractLocalHubAimObservation(result, source.robotToCamera(), alliance, preferredTagId));
            if (observation.isPresent()) {
                return observation;
            }
        }

        return Optional.empty();
    }

    public boolean hasConfiguredHubTagIds(Optional<Alliance> alliance) {
        int[] ids = alliance.orElse(Alliance.Blue) == Alliance.Red
                ? Constants.Vision.RED_HUB_FRONT_TAG_IDS
                : Constants.Vision.BLUE_HUB_FRONT_TAG_IDS;

        for (int id : ids) {
            if (id > 0) {
                return true;
            }
        }
        return false;
    }

    public boolean hasConfiguredPrimaryHubTagId(Optional<Alliance> alliance) {
        return getPreferredHubFrontTagId(alliance) > 0;
    }

    private void collectHubTargets(
            PhotonPipelineResult result,
            Transform3d robotToCamera,
            Optional<Alliance> alliance,
            List<Translation2d> robotToTargets) {
        if (result == null || !result.hasTargets()) {
            return;
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (!isAllianceHubTarget(target.getFiducialId(), alliance)) {
                continue;
            }

            Pose3d targetRelativePose = Pose3d.kZero
                    .transformBy(robotToCamera)
                    .transformBy(target.getBestCameraToTarget());
            robotToTargets.add(targetRelativePose.getTranslation().toTranslation2d());
        }
    }

    private Optional<LocalHubAimObservation> extractLocalHubAimObservation(
            PhotonPipelineResult result,
            Transform3d robotToCamera,
            Optional<Alliance> alliance,
            int preferredTagId) {
        if (result == null || !result.hasTargets()) {
            return Optional.empty();
        }

        Pose3d hubCenter = FieldConstants.Hub.getCenterForAlliance(alliance);
        Optional<Pose3d> preferredTagPose = fieldLayout.getTagPose(preferredTagId);
        if (preferredTagPose.isEmpty()) {
            return Optional.empty();
        }

        Transform3d tagToHubGoal = new Transform3d(preferredTagPose.get(), hubCenter);
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() != preferredTagId) {
                continue;
            }

            Pose3d robotToTagPose = Pose3d.kZero
                    .transformBy(robotToCamera)
                    .transformBy(target.getBestCameraToTarget());
            Pose3d robotToGoalPose = robotToTagPose.transformBy(tagToHubGoal);
            Translation2d robotToGoal = robotToGoalPose.getTranslation().toTranslation2d();
            return Optional.of(new LocalHubAimObservation(
                    robotToGoal,
                    robotToGoal.getNorm(),
                    preferredTagId,
                    "local-tag"));
        }

        return Optional.empty();
    }

    private boolean isAllianceHubTarget(int fiducialId, Optional<Alliance> alliance) {
        int[] ids = alliance.orElse(Alliance.Blue) == Alliance.Red
                ? Constants.Vision.RED_HUB_FRONT_TAG_IDS
                : Constants.Vision.BLUE_HUB_FRONT_TAG_IDS;
        if (!hasConfiguredHubTagIds(alliance)) {
            return false;
        }

        for (int id : ids) {
            if (fiducialId == id) {
                return true;
            }
        }
        return false;
    }

    private int getPreferredHubFrontTagId(Optional<Alliance> alliance) {
        return alliance.orElse(Alliance.Blue) == Alliance.Red
                ? Constants.Vision.RED_HUB_PRIMARY_FRONT_TAG_ID
                : Constants.Vision.BLUE_HUB_PRIMARY_FRONT_TAG_ID;
    }

    private void processCamera(CameraPoseSource source) {
        String label = source.label();
        PhotonCamera camera = source.camera();
        PhotonPoseEstimator estimator = source.estimator();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        putDebugNumber("Vision/" + label + "/UnreadResults", results.size());
        estimator.setReferencePose(drivetrain.getState().Pose);

        if (results.isEmpty()) {
            putDebugBoolean("Vision/" + label + "/HasTarget", false);
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            return;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        PhotonPipelineResult result = filterIgnoredTags(latest);
        setLatestStoredResult(label, result);
        putDebugBoolean("Vision/" + label + "/HasTarget", result.hasTargets());

        if (!result.hasTargets()) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            return;
        }

        if (result.getBestTarget() != null) {
            putDebugNumber("Vision/" + label + "/BestTagId", result.getBestTarget().getFiducialId());
            putDebugNumber("Vision/" + label + "/BestTagAmbiguity", result.getBestTarget().getPoseAmbiguity());
            putDebugNumber(
                    "Vision/" + label + "/BestTagDistanceM",
                    result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
        }

        Optional<EstimatedRobotPose> estimate = estimator.update(result);
        if (estimate.isEmpty()) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", "NoEstimate");
            return;
        }

        double estimateTimestamp = estimate.get().timestampSeconds;
        double nowSec = Timer.getFPGATimestamp();
        double measurementAgeSec = nowSec - estimateTimestamp;
        putDebugNumber("Vision/" + label + "/MeasurementAgeSec", measurementAgeSec);
        if (measurementAgeSec > MAX_MEASUREMENT_AGE_SEC) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", "MeasurementTooOld");
            return;
        }

        if (estimateTimestamp <= getLastAcceptedTimestamp(label)) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", "StaleTimestamp");
            return;
        }

        MeasurementStats measurementStats = getMeasurementStats(estimate.get().targetsUsed);
        publishMeasurementStats(label, measurementStats);
        String qualityRejectReason = getQualityRejectReason(measurementStats);
        if (qualityRejectReason != null) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", qualityRejectReason);
            return;
        }

        MotionStats motionStats = getMotionStats();
        publishMotionStats(label, motionStats);
        String motionRejectReason = getMotionRejectReason(motionStats);
        if (motionRejectReason != null) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", motionRejectReason);
            return;
        }

        boolean allowVisionPoseUpdate = drivetrain.shouldAllowVisionPoseUpdate();
        putDebugBoolean(VISION_POSE_UPDATES_ALLOWED_KEY, allowVisionPoseUpdate);
        putDebugBoolean("Vision/" + label + "/RobotStationary", drivetrain.isEffectivelyStationary());

        String allianceSideRejectReason = getAllianceSideRejectReason(estimate.get().targetsUsed);
        if (allianceSideRejectReason != null) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", allianceSideRejectReason);
            return;
        }

        Pose2d estimatedPose = estimate.get().estimatedPose.toPose2d();
        if (!isInFieldBounds(estimatedPose)) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", "OutOfFieldBounds");
            return;
        }

        if (!ENABLE_POSE_FUSION) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugString("Vision/" + label + "/RejectReason", "FusionDisabled");
            return;
        }

        boolean useVisionRotation = drivetrain.shouldUseVisionRotation();
        boolean forceVisionPose = drivetrain.shouldForceVisionPose();
        putDebugBoolean("Vision/" + label + "/UseVisionRotation", useVisionRotation);
        putDebugBoolean("Vision/" + label + "/ForceVisionPose", forceVisionPose);

        if (!poseSeededFromVision && ENABLE_INITIAL_POSE_SEED) {
            drivetrain.resetPoseFromVision(estimatedPose, useVisionRotation);
            poseSeededFromVision = true;
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", true);
            putDebugString("Vision/" + label + "/RejectReason", "");
            return;
        }

        Pose2d currentPose = drivetrain.getState().Pose;
        double transJump = estimatedPose.getTranslation().getDistance(currentPose.getTranslation());
        double rotJumpDeg = Math.abs(estimatedPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        boolean shouldRecoverPose = shouldRecoverFromLargePoseError(measurementStats, transJump);
        putDebugBoolean("Vision/" + label + "/PoseRecoveryEligible", shouldRecoverPose);
        if (!shouldRecoverPose && (transJump > MAX_TRANSLATION_JUMP_M || rotJumpDeg > MAX_ROTATION_JUMP_DEG)) {
            putDebugBoolean("Vision/" + label + "/EstimateAccepted", false);
            putDebugNumber("Vision/" + label + "/RejectedTransJumpM", transJump);
            putDebugNumber("Vision/" + label + "/RejectedRotJumpDeg", rotJumpDeg);
            putDebugString("Vision/" + label + "/RejectReason", "JumpFilter");
            return;
        }

        Pose2d fusedPose = useVisionRotation
                ? estimatedPose
                : drivetrain.useGyroHeadingForPose(estimatedPose);
        Matrix<N3, N1> stdDevs = getVisionStdDevs(result);

        if (forceVisionPose || shouldHardResetDuringBump(result, transJump) || shouldRecoverPose) {
            drivetrain.resetPoseFromVision(fusedPose, useVisionRotation);
        } else {
            drivetrain.addVisionMeasurement(fusedPose, estimateTimestamp, stdDevs);
        }

        putDebugBoolean("Vision/" + label + "/EstimateAccepted", true);
        putDebugString("Vision/" + label + "/RejectReason", "");
        putDebugNumber("Vision/" + label + "/TagCount", result.targets.size());
        putDebugNumber("Vision/" + label + "/X", estimatedPose.getX());
        putDebugNumber("Vision/" + label + "/Y", estimatedPose.getY());
        putDebugNumber("Vision/" + label + "/ThetaDeg", estimatedPose.getRotation().getDegrees());
        putDebugNumber("Vision/" + label + "/TimestampSec", estimateTimestamp);

        incrementFuseCount(label);
        putDebugNumber("Vision/" + label + "/FuseCount", getFuseCount(label));
        setLastAcceptedTimestamp(label, estimateTimestamp);
    }

    private Matrix<N3, N1> getVisionStdDevs(PhotonPipelineResult result) {
        int tagCount = result.targets.size();
        if (drivetrain.isTraversingBump()) {
            if (tagCount >= 2) {
                return VecBuilder.fill(0.1, 0.1, 0.3);
            }
            return VecBuilder.fill(0.2, 0.2, 0.5);
        }

        if (!drivetrain.isEffectivelyStationary()) {
            if (tagCount >= 2) {
                return VecBuilder.fill(1.8, 1.8, 3.0);
            }
            return VecBuilder.fill(3.0, 3.0, 4.5);
        }

        if (tagCount >= 2) {
            return VecBuilder.fill(0.3, 0.3, 0.7);
        }
        return VecBuilder.fill(0.8, 0.8, 1.4);
    }

    private boolean shouldHardResetDuringBump(PhotonPipelineResult result, double translationJumpMeters) {
        return drivetrain.isTraversingBump()
                && result.targets.size() >= BUMP_HARD_RESET_TAG_COUNT
                && translationJumpMeters >= BUMP_HARD_RESET_TRANSLATION_ERROR_M;
    }

    private boolean shouldRecoverFromLargePoseError(MeasurementStats stats, double translationJumpMeters) {
        return drivetrain.isEffectivelyStationary()
                && translationJumpMeters >= STATIONARY_POSE_RECOVERY_TRANSLATION_ERROR_M
                && (stats.tagCount() >= 2
                        || stats.closestTagDistanceMeters() <= STATIONARY_POSE_RECOVERY_CLOSE_TAG_DISTANCE_M);
    }

    private MeasurementStats getMeasurementStats(List<PhotonTrackedTarget> targets) {
        if (targets == null || targets.isEmpty()) {
            return new MeasurementStats(0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, 1.0);
        }

        double closestDistanceMeters = Double.POSITIVE_INFINITY;
        double farthestDistanceMeters = 0.0;
        double totalDistanceMeters = 0.0;
        double maxAmbiguity = 0.0;

        for (PhotonTrackedTarget target : targets) {
            double tagDistanceMeters = target.getBestCameraToTarget().getTranslation().getNorm();
            closestDistanceMeters = Math.min(closestDistanceMeters, tagDistanceMeters);
            farthestDistanceMeters = Math.max(farthestDistanceMeters, tagDistanceMeters);
            totalDistanceMeters += tagDistanceMeters;
            maxAmbiguity = Math.max(maxAmbiguity, Math.max(0.0, target.getPoseAmbiguity()));
        }

        return new MeasurementStats(
                targets.size(),
                closestDistanceMeters,
                totalDistanceMeters / targets.size(),
                farthestDistanceMeters,
                maxAmbiguity);
    }

    private void publishMeasurementStats(String label, MeasurementStats stats) {
        putDebugNumber("Vision/" + label + "/UsedTagCount", stats.tagCount());
        putDebugNumber("Vision/" + label + "/ClosestTagDistanceM", stats.closestTagDistanceMeters());
        putDebugNumber("Vision/" + label + "/AverageTagDistanceM", stats.averageTagDistanceMeters());
        putDebugNumber("Vision/" + label + "/FarthestTagDistanceM", stats.farthestTagDistanceMeters());
        putDebugNumber("Vision/" + label + "/MaxTagAmbiguity", stats.maxAmbiguity());
    }

    private MotionStats getMotionStats() {
        return new MotionStats(
                drivetrain.getRobotLinearSpeedMetersPerSec(),
                Math.abs(drivetrain.getRobotLinearAccelerationMetersPerSecSq()),
                Math.abs(drivetrain.getRobotAngularAccelerationRadPerSecSq()));
    }

    private void publishMotionStats(String label, MotionStats stats) {
        putDebugNumber("Vision/" + label + "/RobotVelocityMps", stats.robotVelocityMetersPerSec());
        putDebugNumber("Vision/" + label + "/RobotAccelerationMpsSq", stats.robotAccelerationMetersPerSecSq());
        putDebugNumber(
                "Vision/" + label + "/RobotAngularAccelerationRadPerSecSq",
                stats.robotAngularAccelerationRadPerSecSq());
    }

    private String getQualityRejectReason(MeasurementStats stats) {
        if (stats.tagCount() <= 0) {
            return "NoUsableTags";
        }

        boolean robotMoving = !drivetrain.isEffectivelyStationary() && !drivetrain.shouldForceVisionPose();
        if (robotMoving) {
            if (stats.closestTagDistanceMeters() > MAX_MOVING_TAG_DISTANCE_M) {
                return "MovingTagsTooFar";
            }

            if (stats.averageTagDistanceMeters() > MAX_MOVING_AVERAGE_TAG_DISTANCE_M) {
                return "MovingAvgTagDistanceTooFar";
            }

            if (stats.tagCount() == 1 && stats.closestTagDistanceMeters() > MAX_MOVING_SINGLE_TAG_DISTANCE_M) {
                return "MovingSingleTagTooFar";
            }

            if (stats.tagCount() == 1 && stats.maxAmbiguity() > MAX_MOVING_SINGLE_TAG_AMBIGUITY) {
                return "MovingSingleTagAmbiguous";
            }
        }

        if (stats.closestTagDistanceMeters() > MAX_TAG_DISTANCE_M) {
            return "TagsTooFar";
        }

        if (stats.tagCount() == 1 && stats.closestTagDistanceMeters() > MAX_SINGLE_TAG_DISTANCE_M) {
            return "SingleTagTooFar";
        }

        if (stats.tagCount() == 1 && stats.maxAmbiguity() > MAX_SINGLE_TAG_AMBIGUITY) {
            return "SingleTagAmbiguous";
        }

        return null;
    }

    private String getAllianceSideRejectReason(List<PhotonTrackedTarget> targets) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty() || drivetrain.isEffectivelyStationary() || drivetrain.shouldForceVisionPose()) {
            return null;
        }

        double fieldMidX = FieldConstants.FIELD_LENGTH / 2.0;
        double currentX = drivetrain.getState().Pose.getX();
        boolean robotOnBlueHalf = currentX < fieldMidX;
        boolean robotOnRedHalf = currentX > fieldMidX;

        if (!robotOnBlueHalf && !robotOnRedHalf) {
            return null;
        }

        boolean allTagsOnOpponentHalf = true;
        boolean sawAnyKnownTag = false;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }

            sawAnyKnownTag = true;
            boolean tagOnBlueHalf = tagPose.get().getX() < fieldMidX;
            boolean tagOnRedHalf = tagPose.get().getX() > fieldMidX;

            if (alliance.get() == Alliance.Blue && tagOnBlueHalf) {
                allTagsOnOpponentHalf = false;
                break;
            }

            if (alliance.get() == Alliance.Red && tagOnRedHalf) {
                allTagsOnOpponentHalf = false;
                break;
            }
        }

        if (!sawAnyKnownTag) {
            return null;
        }

        if (alliance.get() == Alliance.Blue && robotOnBlueHalf && allTagsOnOpponentHalf) {
            return "OpponentHalfTags";
        }

        if (alliance.get() == Alliance.Red && robotOnRedHalf && allTagsOnOpponentHalf) {
            return "OpponentHalfTags";
        }

        return null;
    }

    private String getMotionRejectReason(MotionStats stats) {
        if (!SmartDashboard.getBoolean(DYNAMICS_FILTER_ENABLED_KEY, true)) {
            return null;
        }

        double maxVelocityMetersPerSec = SmartDashboard.getNumber(
                MAX_ACCEPTABLE_VELOCITY_MPS_KEY,
                DEFAULT_MAX_ACCEPTABLE_VELOCITY_MPS);
        double maxAccelerationMetersPerSecSq = SmartDashboard.getNumber(
                MAX_ACCEPTABLE_ACCEL_MPS2_KEY,
                DEFAULT_MAX_ACCEPTABLE_ACCEL_MPS2);
        double maxAngularAccelerationRadPerSecSq = SmartDashboard.getNumber(
                MAX_ACCEPTABLE_ANGULAR_ACCEL_RADPS2_KEY,
                DEFAULT_MAX_ACCEPTABLE_ANGULAR_ACCEL_RADPS2);

        if (stats.robotVelocityMetersPerSec() > maxVelocityMetersPerSec) {
            return "RobotVelocityTooHigh";
        }

        if (stats.robotAccelerationMetersPerSecSq() > maxAccelerationMetersPerSecSq) {
            return "RobotAccelerationTooHigh";
        }

        if (stats.robotAngularAccelerationRadPerSecSq() > maxAngularAccelerationRadPerSecSq) {
            return "RobotAngularAccelerationTooHigh";
        }

        return null;
    }

    private boolean isInFieldBounds(Pose2d pose) {
        return pose.getX() >= -FIELD_BOUNDS_MARGIN_M
                && pose.getX() <= FieldConstants.FIELD_LENGTH + FIELD_BOUNDS_MARGIN_M
                && pose.getY() >= -FIELD_BOUNDS_MARGIN_M
                && pose.getY() <= FieldConstants.FIELD_WIDTH + FIELD_BOUNDS_MARGIN_M;
    }

    public boolean resetPoseFromVision() {
        debugOutputEnabled = SmartDashboard.getBoolean(DEBUG_OUTPUT_ENABLED_KEY, false);

        Optional<EstimatedRobotPose> best = Optional.empty();

        for (CameraPoseSource source : cameraPoseSources) {
            best = pickBetterEstimate(best, getLatestEstimate(source.camera(), source.estimator()));
        }

        if (best.isEmpty()) {
            putDebugBoolean("Vision/LastResetUsedVision", false);
            return false;
        }

        Pose2d pose = best.get().estimatedPose.toPose2d();
        if (!isInFieldBounds(pose)) {
            putDebugBoolean("Vision/LastResetUsedVision", false);
            return false;
        }

        drivetrain.resetPoseFromVision(pose, drivetrain.shouldUseVisionRotation());
        poseSeededFromVision = true;
        putDebugBoolean("Vision/LastResetUsedVision", true);
        putDebugNumber("Vision/LastResetX", pose.getX());
        putDebugNumber("Vision/LastResetY", pose.getY());
        putDebugNumber("Vision/LastResetThetaDeg", pose.getRotation().getDegrees());
        return true;
    }

    public void markPoseSeededExternally() {
        poseSeededFromVision = true;
    }

    public void allowInitialVisionSeed() {
        poseSeededFromVision = false;
    }

    private Optional<EstimatedRobotPose> getLatestEstimate(PhotonCamera camera, PhotonPoseEstimator estimator) {
        List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
        if (unread.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult latest = filterIgnoredTags(unread.get(unread.size() - 1));
        if (!latest.hasTargets()) {
            return Optional.empty();
        }

        estimator.setReferencePose(drivetrain.getState().Pose);
        Optional<EstimatedRobotPose> estimate = estimator.update(latest);
        if (estimate.isEmpty()) {
            return Optional.empty();
        }

        MeasurementStats measurementStats = getMeasurementStats(estimate.get().targetsUsed);
        return getQualityRejectReason(measurementStats) == null ? estimate : Optional.empty();
    }

    private PhotonPipelineResult filterIgnoredTags(PhotonPipelineResult result) {
        if (result == null || !result.hasTargets()) {
            return result;
        }

        List<PhotonTrackedTarget> filteredTargets = new ArrayList<>(result.targets.size());
        boolean removedIgnoredTag = false;
        for (PhotonTrackedTarget target : result.targets) {
            if (shouldIgnoreTag(target.getFiducialId())) {
                removedIgnoredTag = true;
                continue;
            }
            filteredTargets.add(target);
        }

        if (!removedIgnoredTag) {
            return result;
        }

        return new PhotonPipelineResult(
                result.metadata,
                filteredTargets,
                Optional.empty());
    }

    private boolean shouldIgnoreTag(int fiducialId) {
        return IGNORED_TAG_IDS.contains(fiducialId);
    }

    private Optional<EstimatedRobotPose> pickBetterEstimate(
            Optional<EstimatedRobotPose> currentBest,
            Optional<EstimatedRobotPose> candidate) {
        if (candidate.isEmpty()) {
            return currentBest;
        }

        if (currentBest.isEmpty()) {
            return candidate;
        }

        int candidateTags = candidate.get().targetsUsed.size();
        int bestTags = currentBest.get().targetsUsed.size();
        if (candidateTags > bestTags) {
            return candidate;
        }

        if (candidateTags == bestTags
                && candidate.get().timestampSeconds >= currentBest.get().timestampSeconds) {
            return candidate;
        }

        return currentBest;
    }

    private Optional<PhotonPipelineResult> getLatestStoredResult(String label) {
        return latestResults.getOrDefault(label, Optional.empty());
    }

    private void setLatestStoredResult(String label, PhotonPipelineResult result) {
        latestResults.put(label, Optional.of(result));
    }

    private double getLastAcceptedTimestamp(String label) {
        return lastAcceptedTimestamps.getOrDefault(label, -1.0);
    }

    private void setLastAcceptedTimestamp(String label, double timestampSec) {
        lastAcceptedTimestamps.put(label, timestampSec);
    }

    private void incrementFuseCount(String label) {
        fuseCounts.put(label, getFuseCount(label) + 1);
    }

    private int getFuseCount(String label) {
        return fuseCounts.getOrDefault(label, 0);
    }

    private boolean isDebugOutputEnabled() {
        return debugOutputEnabled;
    }

    private void putDebugBoolean(String key, boolean value) {
        if (isDebugOutputEnabled()) {
            // SmartDashboard.putBoolean(key, value);
        }
    }

    private void putDebugNumber(String key, double value) {
        if (isDebugOutputEnabled()) {
            // SmartDashboard.putNumber(key, value);
        }
    }

    private void putDebugString(String key, String value) {
        if (isDebugOutputEnabled()) {
            // SmartDashboard.putString(key, value);
        }
    }

}
