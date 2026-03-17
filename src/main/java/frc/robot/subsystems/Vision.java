package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Vision extends SubsystemBase {
    public record HubObservation(
            Translation2d robotToHubFrontMeters,
            double rangeToHubCenterMeters,
            int targetCount,
            String source) {
    }

    private final Drivetrain drivetrain;
    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    private final PhotonPoseEstimator leftEstimator;
    private final PhotonPoseEstimator rightEstimator;
    private int leftFuseCount = 0;
    private int rightFuseCount = 0;
    private boolean poseSeededFromVision = false;
    private double lastLeftAcceptedTimestampSec = -1.0;
    private double lastRightAcceptedTimestampSec = -1.0;
    private Optional<PhotonPipelineResult> lastLeftResult = Optional.empty();
    private Optional<PhotonPipelineResult> lastRightResult = Optional.empty();

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        leftCamera = new PhotonCamera(Constants.Vision.LEFT_CAMERA_NAME);
        rightCamera = new PhotonCamera(Constants.Vision.RIGHT_CAMERA_NAME);

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        leftEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.ROBOT_TO_LEFT_CAMERA);
        leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rightEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.ROBOT_TO_RIGHT_CAMERA);
        rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putNumber("Vision/MaxTranslationJumpM", 3.0);
        SmartDashboard.putNumber("Vision/MaxRotationJumpDeg", 60.0);
        SmartDashboard.putBoolean("Vision/EnablePoseFusion", false);
        SmartDashboard.putBoolean("Vision/EnableInitialPoseSeed", false);
        SmartDashboard.putBoolean("Vision/UseVisionRotation", true);
        SmartDashboard.putNumber("Vision/MaxMeasurementAgeSec", 0.5);
        SmartDashboard.putNumber("Vision/FieldBoundsMarginM", 0.3);
        SmartDashboard.putBoolean("Vision/LastResetUsedVision", false);
    }

    @Override
    public void periodic() {
        processCamera(leftCamera, leftEstimator, "Left");
        processCamera(rightCamera, rightEstimator, "Right");

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
        lastLeftResult.ifPresent(result ->
                collectHubTargets(result, Constants.Vision.ROBOT_TO_LEFT_CAMERA, alliance, robotToTargets));
        lastRightResult.ifPresent(result ->
                collectHubTargets(result, Constants.Vision.ROBOT_TO_RIGHT_CAMERA, alliance, robotToTargets));

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

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator, String label) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        SmartDashboard.putNumber("Vision/" + label + "/UnreadResults", results.size());
        estimator.setReferencePose(drivetrain.getState().Pose);

        if (results.isEmpty()) {
            SmartDashboard.putBoolean("Vision/" + label + "/HasTarget", false);
            SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
            return;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        if ("Left".equals(label)) {
            lastLeftResult = Optional.of(latest);
        } else {
            lastRightResult = Optional.of(latest);
        }

        for (PhotonPipelineResult result : results) {
            SmartDashboard.putBoolean("Vision/" + label + "/HasTarget", result.hasTargets());

            if (!result.hasTargets()) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                continue;
            }

            if (result.getBestTarget() != null) {
                SmartDashboard.putNumber("Vision/" + label + "/BestTagId", result.getBestTarget().getFiducialId());
                SmartDashboard.putNumber("Vision/" + label + "/BestTagAmbiguity", result.getBestTarget().getPoseAmbiguity());
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
            if (measurementAgeSec > SmartDashboard.getNumber("Vision/MaxMeasurementAgeSec", 0.5)) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "MeasurementTooOld");
                continue;
            }

            double lastAcceptedTs = "Left".equals(label) ? lastLeftAcceptedTimestampSec : lastRightAcceptedTimestampSec;
            if (estimateTimestamp <= lastAcceptedTs) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "StaleTimestamp");
                continue;
            }

            Pose2d estimatedPose = estimate.get().estimatedPose.toPose2d();
            if (!isInFieldBounds(estimatedPose)) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "OutOfFieldBounds");
                continue;
            }

            if (!SmartDashboard.getBoolean("Vision/EnablePoseFusion", true)) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "FusionDisabled");
                continue;
            }

            if (!poseSeededFromVision && SmartDashboard.getBoolean("Vision/EnableInitialPoseSeed", true)) {
                drivetrain.resetPose(estimatedPose);
                poseSeededFromVision = true;
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", true);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "");
                continue;
            }

            Pose2d currentPose = drivetrain.getState().Pose;
            double maxTransJump = SmartDashboard.getNumber("Vision/MaxTranslationJumpM", 3.0);
            double maxRotJumpDeg = SmartDashboard.getNumber("Vision/MaxRotationJumpDeg", 60.0);
            double transJump = estimatedPose.getTranslation().getDistance(currentPose.getTranslation());
            double rotJumpDeg = Math.abs(estimatedPose.getRotation().minus(currentPose.getRotation()).getDegrees());
            if (transJump > maxTransJump || rotJumpDeg > maxRotJumpDeg) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                SmartDashboard.putNumber("Vision/" + label + "/RejectedTransJumpM", transJump);
                SmartDashboard.putNumber("Vision/" + label + "/RejectedRotJumpDeg", rotJumpDeg);
                SmartDashboard.putString("Vision/" + label + "/RejectReason", "JumpFilter");
                continue;
            }

            boolean useVisionRotation = SmartDashboard.getBoolean("Vision/UseVisionRotation", true);
            Pose2d fusedPose = useVisionRotation
                    ? estimatedPose
                    : new Pose2d(estimatedPose.getTranslation(), drivetrain.getState().Pose.getRotation());
            Matrix<N3, N1> stdDevs = getVisionStdDevs(result);
            drivetrain.addVisionMeasurement(fusedPose, estimateTimestamp, stdDevs);
            SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", true);
            SmartDashboard.putString("Vision/" + label + "/RejectReason", "");

            SmartDashboard.putNumber("Vision/" + label + "/TagCount", result.targets.size());
            SmartDashboard.putNumber("Vision/" + label + "/X", estimatedPose.getX());
            SmartDashboard.putNumber("Vision/" + label + "/Y", estimatedPose.getY());
            SmartDashboard.putNumber("Vision/" + label + "/ThetaDeg", estimatedPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/" + label + "/TimestampSec", estimateTimestamp);
            if ("Left".equals(label)) {
                leftFuseCount++;
                SmartDashboard.putNumber("Vision/Left/FuseCount", leftFuseCount);
                lastLeftAcceptedTimestampSec = estimateTimestamp;
            } else {
                rightFuseCount++;
                SmartDashboard.putNumber("Vision/Right/FuseCount", rightFuseCount);
                lastRightAcceptedTimestampSec = estimateTimestamp;
            }
        }
    }

    private Matrix<N3, N1> getVisionStdDevs(PhotonPipelineResult result) {
        int tagCount = result.targets.size();
        if (tagCount >= 2) {
            return VecBuilder.fill(0.4, 0.4, 0.8);
        }
        return VecBuilder.fill(1.0, 1.0, 2.0);
    }

    private boolean isInFieldBounds(Pose2d pose) {
        double marginMeters = SmartDashboard.getNumber("Vision/FieldBoundsMarginM", 0.3);
        return pose.getX() >= -marginMeters
                && pose.getX() <= FieldConstants.FIELD_LENGTH + marginMeters
                && pose.getY() >= -marginMeters
                && pose.getY() <= FieldConstants.FIELD_WIDTH + marginMeters;
    }

    public boolean resetPoseFromVision() {
        var leftEstimate = getLatestEstimate(leftCamera, leftEstimator);
        var rightEstimate = getLatestEstimate(rightCamera, rightEstimator);

        Optional<EstimatedRobotPose> best = Optional.empty();
        if (leftEstimate.isPresent() && rightEstimate.isPresent()) {
            int leftTags = leftEstimate.get().targetsUsed.size();
            int rightTags = rightEstimate.get().targetsUsed.size();
            if (leftTags != rightTags) {
                best = leftTags > rightTags ? leftEstimate : rightEstimate;
            } else {
                best = leftEstimate.get().timestampSeconds >= rightEstimate.get().timestampSeconds
                        ? leftEstimate
                        : rightEstimate;
            }
        } else if (leftEstimate.isPresent()) {
            best = leftEstimate;
        } else if (rightEstimate.isPresent()) {
            best = rightEstimate;
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

        drivetrain.resetPose(pose);
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
        return estimator.update(latest);
    }
}
