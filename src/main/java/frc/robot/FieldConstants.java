package frc.robot;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.BallConstants;
import com.techhounds.houndutil.houndlib.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.22);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317.69);

    public static Pose2d rotateBluePoseIfNecessary(Pose2d original) {
        return Utils.shouldFlipValueToRed()
                ? Reflector.rotatePoseAcrossField(original, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH)
                : original;
    }

    public static final BallConstants BALL_CONSTANTS = new BallConstants(
            Grams.of(210).in(Kilograms), Inches.of(3).in(Meters), 1.2, 0.30, 1.2, 0.35, 9.81, 20);

    public static final class Hub {
        // Blue hub target used by SCORE mode.
        // Editing this pose moves where all blue-alliance hub shots aim:
        // X/Y shift left-right/forward-back on field, Z changes required hood/shot arc.
        // Keep in meters; these are converted from official inch measurements.
        public static final Pose3d BLUE_CENTER = new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);

        public static final Pose3d BLUE_FRONT_FACE = new Pose3d(Units.inchesToMeters(158.50), Units.inchesToMeters(158.84),
                Units.inchesToMeters(0), Rotation3d.kZero);

        // Red hub is the blue hub reflected across the field center (180 deg rotation).
        public static final Pose3d RED_CENTER = new Pose3d(
                FIELD_LENGTH - BLUE_CENTER.getX(),
                FIELD_WIDTH - BLUE_CENTER.getY(),
                BLUE_CENTER.getZ(),
                Rotation3d.kZero);

        // Red hub is the blue hub reflected across the field center (180 deg rotation).
        public static final Pose3d RED_FRONT_FACE = new Pose3d(
                FIELD_LENGTH - BLUE_FRONT_FACE.getX(),
                FIELD_WIDTH - BLUE_FRONT_FACE.getY(),
                BLUE_FRONT_FACE.getZ(),
                Rotation3d.kZero);

        public static final double FRONT_FACE_TO_CENTER_XY_METERS = BLUE_CENTER.toPose2d()
                .getTranslation()
                .getDistance(BLUE_FRONT_FACE.toPose2d().getTranslation());

        // Backward-compatibility alias for existing callsites.
        public static final Pose3d CENTER = BLUE_CENTER;

        public static Pose3d getCenterForAlliance(Optional<Alliance> alliance) {
            return alliance.orElse(Alliance.Blue) == Alliance.Red ? RED_CENTER : BLUE_CENTER;
        }

        public static Pose3d getFrontFaceForAlliance(Optional<Alliance> alliance) {
            return alliance.orElse(Alliance.Blue) == Alliance.Red ? RED_FRONT_FACE : BLUE_FRONT_FACE;
        }

        public static Pose3d getCenterForAllianceOrNearest(Optional<Alliance> alliance, Pose2d robotPose) {
            if (alliance.isPresent()) {
                return getCenterForAlliance(alliance);
            }
            double blueDist = robotPose.getTranslation().getDistance(BLUE_CENTER.toPose2d().getTranslation());
            double redDist = robotPose.getTranslation().getDistance(RED_CENTER.toPose2d().getTranslation());
            return redDist < blueDist ? RED_CENTER : BLUE_CENTER;
        }
    }

    public static final class Passing {
        private static final double BLUE_PASSING_X = Hub.BLUE_CENTER.getX() / 2.0;
        private static final double RED_PASSING_X = (FIELD_LENGTH + Hub.RED_CENTER.getX()) / 2.0;
        private static final double LOWER_PASSING_Y = Hub.BLUE_CENTER.getY() / 2.0;
        private static final double UPPER_PASSING_Y = (FIELD_WIDTH + Hub.BLUE_CENTER.getY()) / 2.0;

        public static final Pose3d BLUE_LEFT_TARGET = new Pose3d(
                BLUE_PASSING_X,
                UPPER_PASSING_Y,
                Hub.BLUE_CENTER.getZ(),
                Rotation3d.kZero);
        public static final Pose3d BLUE_RIGHT_TARGET = new Pose3d(
                BLUE_PASSING_X,
                LOWER_PASSING_Y,
                Hub.BLUE_CENTER.getZ(),
                Rotation3d.kZero);

        public static final Pose3d RED_LEFT_TARGET = new Pose3d(
                RED_PASSING_X,
                UPPER_PASSING_Y,
                Hub.RED_CENTER.getZ(),
                Rotation3d.kZero);
        public static final Pose3d RED_RIGHT_TARGET = new Pose3d(
                RED_PASSING_X,
                LOWER_PASSING_Y,
                Hub.RED_CENTER.getZ(),
                Rotation3d.kZero);

        private static final Pose3d[] BLUE_TARGETS = new Pose3d[] { BLUE_LEFT_TARGET, BLUE_RIGHT_TARGET };
        private static final Pose3d[] RED_TARGETS = new Pose3d[] { RED_LEFT_TARGET, RED_RIGHT_TARGET };

        public static Pose3d[] getTargetsForAlliance(Optional<Alliance> alliance) {
            return alliance.orElse(Alliance.Blue) == Alliance.Red ? RED_TARGETS : BLUE_TARGETS;
        }

        public static Pose3d getClosestForAlliance(Pose2d robotPose, Optional<Alliance> alliance) {
            Pose3d[] targets = getTargetsForAlliance(alliance);
            Pose3d closest = targets[0];
            double closestDistance = robotPose.getTranslation().getDistance(closest.toPose2d().getTranslation());

            for (int i = 1; i < targets.length; i++) {
                Pose3d candidate = targets[i];
                double candidateDistance = robotPose.getTranslation().getDistance(candidate.toPose2d().getTranslation());
                if (candidateDistance < closestDistance) {
                    closest = candidate;
                    closestDistance = candidateDistance;
                }
            }
            return closest;
        }

        public static Pose3d getClosestForAllianceOrNearest(Pose2d robotPose, Optional<Alliance> alliance) {
            if (alliance.isPresent()) {
                return getClosestForAlliance(robotPose, alliance);
            }
            Pose3d[] allTargets = new Pose3d[] {
                    BLUE_LEFT_TARGET, BLUE_RIGHT_TARGET, RED_LEFT_TARGET, RED_RIGHT_TARGET
            };
            Pose3d closest = allTargets[0];
            double closestDistance = robotPose.getTranslation().getDistance(closest.toPose2d().getTranslation());
            for (int i = 1; i < allTargets.length; i++) {
                Pose3d candidate = allTargets[i];
                double candidateDistance = robotPose.getTranslation().getDistance(candidate.toPose2d().getTranslation());
                if (candidateDistance < closestDistance) {
                    closest = candidate;
                    closestDistance = candidateDistance;
                }
            }
            return closest;
        }
    }

    public static final class ShotZones {
        public static boolean isPastAllianceHub(Pose2d robotPose, Optional<Alliance> alliance) {
            if (alliance.orElse(Alliance.Blue) == Alliance.Red) {
                return robotPose.getX() < Hub.RED_CENTER.getX();
            }
            return robotPose.getX() > Hub.BLUE_CENTER.getX();
        }

        public static boolean isInPassingZone(Pose2d robotPose, Optional<Alliance> alliance) {
            return isPastAllianceHub(robotPose, alliance);
        }

        public static boolean isInScoringZone(Pose2d robotPose, Optional<Alliance> alliance) {
            return !isPastAllianceHub(robotPose, alliance);
        }
    }
}
