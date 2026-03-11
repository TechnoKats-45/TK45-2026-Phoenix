package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.TreeMap;

public class Constants {
    
    public static final int CONFIG_RETRIES = 3;

    public class CAN_BUS
    {
        public static final String RIO = "rio";
        public static final String CANIVORE = "canivore";
    }

    public class CAN_ID 
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

        public static final int INTAKE_LEFT_PIVOT = 50;
        public static final int INTAKE_RIGHT_PIVOT = 51;
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
    
    public class Intake
    {
        public static final double INTAKE_SPEED = 0.5; // TODO - TUNE
        public static final double INTAKE_ROTATE_SPEED = 0.5; // TODO - TUNE
        public static final double ANGLE_TOLERANCE_DEGREES = 1; // TODO - TUNE
        public static final double PIVOT_ANGLE_DOWN = 0.0;
        public static final double PIVOT_ANGLE_UP_STOWED = 9; // TODO - tune
        public static final double SPEED_TOLERANCE_RPS = 0.5; // TODO - TUNE
    }

    public class Shooter 
    {

    }

    public class Hood 
    {

    }

    public class Floor 
    {
        public static final double FLOOR_SPEED = 0.5; // TODO - TUNE
        public static final double SPEED_TOLERANCE_RPS = 0.5; // TODO - TUNE
    }
    
    
    public class Feeder 
    {

    }

    public class Vision 
    {
        
    }
}




