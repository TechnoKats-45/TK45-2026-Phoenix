package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.TreeMap;

public class Constants 
{
    
    public static final int CONFIG_RETRIES = 3;

    public static class CAN_BUS
    {
        public static final String RIO = "rio";
        public static final String CANIVORE = "canivore";
    }

    public static class CAN_ID 
    {
        public static final int PIGEON = 2;

        public static final int FRONT_RIGHT_DRIVE = 10;     // Stored in Tuner Constants
        public static final int FRONT_RIGHT_STEER = 11;     // Stored in Tuner Constants
        public static final int FRONT_RIGHT_ENCODER = 12;   // Stored in Tuner Constants

        public static final int BACK_RIGHT_DRIVE = 20;      // Stored in Tuner Constants
        public static final int BACK_RIGHT_STEER = 21;      // Stored in Tuner Constants
        public static final int BACK_RIGHT_ENCODER = 22;    // Stored in Tuner Constants

        public static final int BACK_LEFT_DRIVE = 30;       // Stored in Tuner Constants
        public static final int BACK_LEFT_STEER = 31;       // Stored in Tuner Constants
        public static final int BACK_LEFT_ENCODER = 32;     // Stored in Tuner Constants

        public static final int FRONT_LEFT_DRIVE = 40;      // Stored in Tuner Constants
        public static final int FRONT_LEFT_STEER = 41;      // Stored in Tuner Constants
        public static final int FRONT_LEFT_ENCODER = 42;    // Stored in Tuner Constants

        public static final int INTAKE_PIVOT = 50;
        public static final int INTAKE_LEFT_ROLLER = 52;
        public static final int INTAKE_RIGHT_ROLLER = 53;

        public static final int LEFT_FLOOR_ROLLER = 54;
        public static final int RIGHT_FLOOR_ROLLER = 55;

        public static final int HOOD = 56; 

        public static final int LEFT_FEEDER = 57;
        public static final int RIGHT_FEEDER = 58;

        public static final int LEFT_SHOOTER = 59;
        public static final int LEFT_MIDDLE_SHOOTER = 60;
        public static final int RIGHT_MIDDLE_SHOOTER = 61;
        public static final int RIGHT_SHOOTER = 62;
    }
    
    public static class Intake
    {
        public static final double INTAKE_SPEED = 1;
        public static final double SPEED_TOLERANCE_RPS = 0.5;

        public static final double INTAKE_ROTATE_SPEED = 0.5;
        public static final double ANGLE_TOLERANCE_DEGREES = 1.0;

        public static final double PIVOT_ANGLE_DOWN = 0.0;
        public static final double PIVOT_ANGLE_UP_STOWED = -75.498046875;
        public static final double halfStow = PIVOT_ANGLE_UP_STOWED*1/2;
    }

    public static class Shooter 
    {
        public static final double MAX_SPEED_RPS = 100.0;
        public static final double SPEED_TOLERANCE_RPS = 2.0;

        public static final double SHOOTER_SPEED_CLOSE = 60.0; // TODO - TUNE 

        public record ShotProfile(double hoodDeg, double speedRps) {}

        public static final TreeMap<Double, ShotProfile> DISTANCE_ANGLE_SPEED = new TreeMap<>();
        static 
        {
            // Format: distance_inches -> ShotProfile(hood_deg, speed_rps)
            // Example: DISTANCE_ANGLE_SPEED.put(120.0, new ShotProfile(22.0, 70.0));
            DISTANCE_ANGLE_SPEED.put(87.8,new ShotProfile(Hood.MIN_ANGLE,49));
            DISTANCE_ANGLE_SPEED.put(110.0, new ShotProfile(Hood.MIN_ANGLE,53.5));
            DISTANCE_ANGLE_SPEED.put(150.0, new ShotProfile(Hood.MIN_ANGLE, 70.0));
        }

        public static ShotProfile getShotProfileForDistanceInches(double distanceInches) {
            if (DISTANCE_ANGLE_SPEED.isEmpty()) {
                return new ShotProfile(0.0, 0.0);
            }

            var lower = DISTANCE_ANGLE_SPEED.floorEntry(distanceInches);
            var upper = DISTANCE_ANGLE_SPEED.ceilingEntry(distanceInches);

            if (lower == null) {
                return upper.getValue();
            }
            if (upper == null) {
                return lower.getValue();
            }
            if (lower.getKey().equals(upper.getKey())) {
                return lower.getValue();
            }

            double ratio = (distanceInches - lower.getKey()) / (upper.getKey() - lower.getKey());
            double hoodDeg = lower.getValue().hoodDeg()
                    + ratio * (upper.getValue().hoodDeg() - lower.getValue().hoodDeg());
            double speedRps = lower.getValue().speedRps()
                    + ratio * (upper.getValue().speedRps() - lower.getValue().speedRps());
            return new ShotProfile(hoodDeg, speedRps);
        }
    }

    public static class Hood 
    {
        public static final double ROTATE_SPEED = 0.5; // TODO - TUNE
        public static final double ANGLE_TOLERANCE_DEGREES = 1; // TODO - TUNE
        public static final double MIN_ANGLE = 14.653417;
        public static final double MAX_ANGLE = 34.900662;
        public static final double HOOD_ANGLE_DOWN = MIN_ANGLE;
        public static final double HOOD_ANGLE_UP_STOWED = MAX_ANGLE;
    }
    
    public static class Floor 
    {
        public static final double SPEED_TOLERANCE_RPS = 0.5; // TODO - TUNE
        public static final double FLOOR_SPEED = 0.5; // TODO - TUNE
        public static final double SHOOT_SPEED = 0.5; // TODO - TUNE
        public static final double AUTO_FEED_SPEED = 0.5;  // TODO - TUNE
    }
    
    public static class Feeder 
    {
        public static final double SPEED_TOLERANCE_RPS = 0.5; // TODO - TUNE
        public static final double FEEDER_SPEED = 0.5; // TODO - TUNE
        public static final double SHOOT_SPEED = 0.5; // TODO - TUNE
        public static final double AUTO_FEED_SPEED = 0.25; //TDOD - TUNE
    }

    public static class Vision
    {
        // Match these exactly to PhotonVision camera names on the coprocessor.
        public static final String LEFT_CAMERA_NAME = "Photon_Left";
        public static final String RIGHT_CAMERA_NAME = "Photon_Right";
        public static final String SIDE_LEFT_CAMERA_NAME = "Photon_Side_Left";
        public static final String SIDE_RIGHT_CAMERA_NAME = "Photon_Side_Right";

        // While these remain non-positive, AutoAim will use any visible fiducial target
        // from the front shooter cameras before falling back to pose-only aiming.
        public static final int[] BLUE_HUB_FRONT_TAG_IDS = {25,26};
        public static final int[] RED_HUB_FRONT_TAG_IDS = {9,10};
        public static final int BLUE_HUB_PRIMARY_FRONT_TAG_ID = 26;
        public static final int RED_HUB_PRIMARY_FRONT_TAG_ID = 10;

        public static final double AUTO_AIM_TRANSLATION_DEADBAND = 0.1;

        public static final double AUTO_AIM_ROTATION_TOLERANCE_DEG = 2.0;
        public static final double AUTO_AIM_MAX_ROTATION_RATE_SCALE = .75;
        public static final double AUTO_AIM_MAX_ANGULAR_RATE_RPS = 1.5;

        // Robot-to-camera transforms using measured offsets from robot center.
        // Input measurements were inches; converted here to meters.
        public static final Transform3d ROBOT_TO_LEFT_CAMERA = new Transform3d(
                new Translation3d(
                        -13.265234 * 0.0254,
                        5.229871 * 0.0254,
                        8.127905 * 0.0254),
                new Rotation3d(
                        0.0,
                        Math.toRadians(10.0),
                        Math.toRadians(-170.0)));

        public static final Transform3d ROBOT_TO_RIGHT_CAMERA = new Transform3d(
                new Translation3d(
                        -13.265234 * 0.0254,
                        -5.229871 * 0.0254,
                        8.127905 * 0.0254),
                new Rotation3d(
                        0.0,
                        Math.toRadians(10.0),
                        Math.toRadians(170.0)));

        public static final Transform3d ROBOT_TO_SIDE_LEFT_CAMERA = new Transform3d(
                new Translation3d(
                        -1.885676 * 0.0254,
                        13.345555 * 0.0254,
                        8.589373 * 0.0254),
                new Rotation3d(
                        Math.toRadians(0.0),
                        Math.toRadians(10.0),
                        Math.toRadians(90.0)));

        public static final Transform3d ROBOT_TO_SIDE_RIGHT_CAMERA = new Transform3d(
                new Translation3d(
                        -1.885676 * 0.0254,
                        -13.345555 * 0.0254,
                        8.589373 * 0.0254),
                new Rotation3d(
                        Math.toRadians(0.0),
                        Math.toRadians(10.0),
                        Math.toRadians(-90.0)));
    }
}




