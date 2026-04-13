package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

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

    public record HubObservation(
            Translation2d robotToHubFrontMeters,
            double rangeToHubCenterMeters,
            int targetCount,
            String source) {
    }

    private final Drivetrain drivetrain;
    private final List<CameraPoseSource> cameraPoseSources;
    private final Map<String, Optional<PhotonPipelineResult>> latestResults = new HashMap<>();
    private final Map<String, Double> lastAcceptedTimestamps = new HashMap<>();
    private final Map<String, Integer> fuseCounts = new HashMap<>();
    private boolean poseSeededFromVision = false;

    // Vision fusion tuning constants (hardcoded; not configurable via SmartDashboard)
    private static final double MAX_TRANSLATION_JUMP_M = 6.0;
    private static final double MAX_ROTATION_JUMP_DEG = 120.0;
    private static final boolean ENABLE_POSE_FUSION = true;
    private static final boolean ENABLE_INITIAL_POSE_SEED = true;
    private static final double MAX_MEASUREMENT_AGE_SEC = 0.5;
    private static final double FIELD_BOUNDS_MARGIN_M = 0.3;
    private static final double BUMP_HARD_RESET_TRANSLATION_ERROR_M = 1.0;
    private static final int BUMP_HARD_RESET_TAG_COUNT = 2;
    private static final double MAX_TAG_DISTANCE_M = 4.5;
    private static final double MAX_SINGLE_TAG_DISTANCE_M = 3.0;
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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

        SmartDashboard.putBoolean("Vision/LastResetUsedVision", false);
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
        for (CameraPoseSource source : cameraPoseSources) {
            processCamera(source);
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose3d hubCenter = FieldConstants.Hub.getCenterForAllianceOrNearest(alliance, drivetrain.getState().Pose);
        double poseDistanceMeters = drivetrain.getState().Pose.getTranslation()
                .getDistance(hubCenter.toPose2d().getTranslation());
        Optional<HubObservation> hubObservation = getHubObservation(alliance);

        SmartDashboard.putString("Vision/Alliance", alliance.map(Enum::name).orElse("Unknown"));
        SmartDashboard.putNumber("Vision/TargetDistanceMeters", poseDistanceMeters);
        SmartDashboard.putNumber("Vision/TargetDistanceInches", Units.metersToInches(poseDistanceMeters));
        SmartDashboard.putBoolean("Vision/HasObservedTargetDistance", hubObservation.isPresent());
        SmartDashboard.putNumber(
                "Vision/ObservedTargetDistanceMeters",
                hubObservation.map(HubObservation::rangeToHubCenterMeters).orElse(-1.0));
        SmartDashboard.putNumber(
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

    private void processCamera(CameraPoseSource source) {
        String label = source.label();
        PhotonCamera camera = source.camera();
        PhotonPoseEstimator estimator = source.estimator();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        SmartDashboard.putNumber("Vision/" + label + "/UnreadResults", results.size());
        estimator.setReferencePose(drivetrain.getState().Pose);

        if (results.isEmpty()) {
            SmartDashboard.putBoolean("Vision/" + label + "/HasTarget", false);
            SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
            return;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        setLatestStoredResult(label, latest);

        for (PhotonPipelineResult result : results) {
            SmartDashboard.putBoolean("Vision/" + label + "/HasTarget", result.hasTargets());

            if (!result.hasTargets()) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                continue;
            }

            if (result.getBestTarget() != null) {
                SmartDashboard.putNumber("Vision/" + label + "/BestTagId", result.getBestTarget().getFiducialId());
                SmartDashboard.putNumber("Vision/" + label + "/BestTagAmbiguity", result.getBestTarget().getPoseAmbiguity());
                SmartDashboard.putNumber(
                        "Vision/" + label + "/BestTagDistanceM",
                        result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
            }

            Optional<EstimatedRobotPose> estimate = estimator.update(result);
            if (estimate.isEmpty()) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "NoEstimate");
                continue;
            }

            double estimateTimestamp = estimate.get().timestampSeconds;
            double nowSec = Timer.getFPGATimestamp();
            double measurementAgeSec = nowSec - estimateTimestamp;
            SmartDashboard.putNumber("Vision/" + label + "/MeasurementAgeSec", measurementAgeSec);
            if (measurementAgeSec > MAX_MEASUREMENT_AGE_SEC) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "MeasurementTooOld");
                continue;
            }

            if (estimateTimestamp <= getLastAcceptedTimestamp(label)) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "StaleTimestamp");
                continue;
            }

            MeasurementStats measurementStats = getMeasurementStats(estimate.get().targetsUsed);
            publishMeasurementStats(label, measurementStats);
            String qualityRejectReason = getQualityRejectReason(measurementStats);
            if (qualityRejectReason != null) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", qualityRejectReason);
                continue;
            }

            Pose2d estimatedPose = estimate.get().estimatedPose.toPose2d();
            if (!isInFieldBounds(estimatedPose)) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "OutOfFieldBounds");
                continue;
            }

            if (!ENABLE_POSE_FUSION) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "FusionDisabled");
                continue;
            }

            boolean useVisionRotation = drivetrain.shouldUseVisionRotation();
            SmartDashboard.putBoolean("Vision/" + label + "/UseVisionRotation", useVisionRotation);

            if (!poseSeededFromVision && ENABLE_INITIAL_POSE_SEED) {
                drivetrain.resetPoseFromVision(estimatedPose, useVisionRotation);
                poseSeededFromVision = true;
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", true);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "");
                continue;
            }

            Pose2d currentPose = drivetrain.getState().Pose;
            double transJump = estimatedPose.getTranslation().getDistance(currentPose.getTranslation());
            double rotJumpDeg = Math.abs(estimatedPose.getRotation().minus(currentPose.getRotation()).getDegrees());
            if (transJump > MAX_TRANSLATION_JUMP_M || rotJumpDeg > MAX_ROTATION_JUMP_DEG) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putNumber("Vision/" + label + "/RejectedTransJumpM", transJump);
                SmartDashboard.putNumber("Vision/" + label + "/RejectedRotJumpDeg", rotJumpDeg);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "JumpFilter");
                continue;
            }

            Pose2d fusedPose = useVisionRotation
                    ? estimatedPose
                    : drivetrain.useGyroHeadingForPose(estimatedPose);
            Matrix<N3, N1> stdDevs = getVisionStdDevs(result);

            if (shouldHardResetDuringBump(result, transJump)) {
                drivetrain.resetPoseFromVision(fusedPose, useVisionRotation);
            } else {
                drivetrain.addVisionMeasurement(fusedPose, estimateTimestamp, stdDevs);
            }

            SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", true);
            SmartDashboard.putString("Vision/" + label + "/RejectReason", "");
            SmartDashboard.putNumber("Vision/" + label + "/TagCount", result.targets.size());
            SmartDashboard.putNumber("Vision/" + label + "/X", estimatedPose.getX());
            SmartDashboard.putNumber("Vision/" + label + "/Y", estimatedPose.getY());
            SmartDashboard.putNumber("Vision/" + label + "/ThetaDeg", estimatedPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/" + label + "/TimestampSec", estimateTimestamp);

            incrementFuseCount(label);
            SmartDashboard.putNumber("Vision/" + label + "/FuseCount", getFuseCount(label));
            setLastAcceptedTimestamp(label, estimateTimestamp);
        }
    }

    private Matrix<N3, N1> getVisionStdDevs(PhotonPipelineResult result) {
        int tagCount = result.targets.size();
        if (drivetrain.isTraversingBump()) {
            if (tagCount >= 2) {
                return VecBuilder.fill(0.1, 0.1, 0.3);
            }
            return VecBuilder.fill(0.2, 0.2, 0.5);
        }

        if (tagCount >= 2) {
            return VecBuilder.fill(0.2, 0.2, 0.5);
        }
        return VecBuilder.fill(0.6, 0.6, 1.0);
    }

    private boolean shouldHardResetDuringBump(PhotonPipelineResult result, double translationJumpMeters) {
        return drivetrain.isTraversingBump()
                && result.targets.size() >= BUMP_HARD_RESET_TAG_COUNT
                && translationJumpMeters >= BUMP_HARD_RESET_TRANSLATION_ERROR_M;
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
        SmartDashboard.putNumber("Vision/" + label + "/UsedTagCount", stats.tagCount());
        SmartDashboard.putNumber("Vision/" + label + "/ClosestTagDistanceM", stats.closestTagDistanceMeters());
        SmartDashboard.putNumber("Vision/" + label + "/AverageTagDistanceM", stats.averageTagDistanceMeters());
        SmartDashboard.putNumber("Vision/" + label + "/FarthestTagDistanceM", stats.farthestTagDistanceMeters());
        SmartDashboard.putNumber("Vision/" + label + "/MaxTagAmbiguity", stats.maxAmbiguity());
    }

    private String getQualityRejectReason(MeasurementStats stats) {
        if (stats.tagCount() <= 0) {
            return "NoUsableTags";
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

    private boolean isInFieldBounds(Pose2d pose) {
        return pose.getX() >= -FIELD_BOUNDS_MARGIN_M
                && pose.getX() <= FieldConstants.FIELD_LENGTH + FIELD_BOUNDS_MARGIN_M
                && pose.getY() >= -FIELD_BOUNDS_MARGIN_M
                && pose.getY() <= FieldConstants.FIELD_WIDTH + FIELD_BOUNDS_MARGIN_M;
    }

    public boolean resetPoseFromVision() {
        Optional<EstimatedRobotPose> best = Optional.empty();

        for (CameraPoseSource source : cameraPoseSources) {
            best = pickBetterEstimate(best, getLatestEstimate(source.camera(), source.estimator()));
        }

        if (best.isEmpty()) {
            SmartDashboard.putBoolean("Vision/LastResetUsedVision", false);
            return false;
        }

        Pose2d pose = best.get().estimatedPose.toPose2d();
        if (!isInFieldBounds(pose)) {
            SmartDashboard.putBoolean("Vision/LastResetUsedVision", false);
            return false;
        }

        drivetrain.resetPoseFromVision(pose, drivetrain.shouldUseVisionRotation());
        poseSeededFromVision = true;
        SmartDashboard.putBoolean("Vision/LastResetUsedVision", true);
        SmartDashboard.putNumber("Vision/LastResetX", pose.getX());
        SmartDashboard.putNumber("Vision/LastResetY", pose.getY());
        SmartDashboard.putNumber("Vision/LastResetThetaDeg", pose.getRotation().getDegrees());
        return true;
    }

    private Optional<EstimatedRobotPose> getLatestEstimate(PhotonCamera camera, PhotonPoseEstimator estimator) {
        List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
        if (unread.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult latest = unread.get(unread.size() - 1);
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

}
