package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.json.simple.parser.ParseException;

public class Telemetry {
    private final double MaxSpeed;
    private final frc.robot.subsystems.Drivetrain drivetrain;
    private static final String[] DRIVE_CURRENT_KEYS = new String[] {
        "Swerve/FrontLeftDriveCurrentAmps",
        "Swerve/FrontRightDriveCurrentAmps",
        "Swerve/BackLeftDriveCurrentAmps",
        "Swerve/BackRightDriveCurrentAmps"
    };

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed, frc.robot.subsystems.Drivetrain drivetrain) {
        MaxSpeed = maxSpeed;
        this.drivetrain = drivetrain;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final DoubleArrayPublisher blueHubPub = table.getDoubleArrayTopic("blueHub").publish();
    private final DoubleArrayPublisher redHubPub = table.getDoubleArrayTopic("redHub").publish();
    private final DoubleArrayPublisher bluePassingTargetsPub = table.getDoubleArrayTopic("bluePassingTargets").publish();
    private final DoubleArrayPublisher redPassingTargetsPub = table.getDoubleArrayTopic("redPassingTargets").publish();
    private final DoubleArrayPublisher blueScoringZonePub = table.getDoubleArrayTopic("blueScoringZone").publish();
    private final DoubleArrayPublisher bluePassingZonePub = table.getDoubleArrayTopic("bluePassingZone").publish();
    private final DoubleArrayPublisher redScoringZonePub = table.getDoubleArrayTopic("redScoringZone").publish();
    private final DoubleArrayPublisher redPassingZonePub = table.getDoubleArrayTopic("redPassingZone").publish();
    private final DoubleArrayPublisher trajectoryPub = table.getDoubleArrayTopic("trajectory").publish();
    private final DoubleArrayPublisher startingPosePub = table.getDoubleArrayTopic("startingPose").publish();
    private final StringPublisher selectedAutoNamePub = table.getStringTopic("selectedAutoName").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_blueHubArray = posesToFieldObjectArray(FieldConstants.Hub.BLUE_CENTER);
    private final double[] m_redHubArray = posesToFieldObjectArray(FieldConstants.Hub.RED_CENTER);
    private final double[] m_bluePassingTargetsArray = posesToFieldObjectArray(
            FieldConstants.Passing.BLUE_LEFT_TARGET,
            FieldConstants.Passing.BLUE_RIGHT_TARGET);
    private final double[] m_redPassingTargetsArray = posesToFieldObjectArray(
            FieldConstants.Passing.RED_LEFT_TARGET,
            FieldConstants.Passing.RED_RIGHT_TARGET);
    private final double[] m_blueScoringZoneArray = posesToFieldObjectArray(
            new Pose3d(0.0, 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.BLUE_CENTER.getX(), 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.BLUE_CENTER.getX(), FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(0.0, FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(0.0, 0.0, 0.0, Pose3d.kZero.getRotation()));
    private final double[] m_bluePassingZoneArray = posesToFieldObjectArray(
            new Pose3d(FieldConstants.Hub.BLUE_CENTER.getX(), 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.FIELD_LENGTH, 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.BLUE_CENTER.getX(), FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.BLUE_CENTER.getX(), 0.0, 0.0, Pose3d.kZero.getRotation()));
    private final double[] m_redPassingZoneArray = posesToFieldObjectArray(
            new Pose3d(0.0, 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.RED_CENTER.getX(), 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.RED_CENTER.getX(), FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(0.0, FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(0.0, 0.0, 0.0, Pose3d.kZero.getRotation()));
    private final double[] m_redScoringZoneArray = posesToFieldObjectArray(
            new Pose3d(FieldConstants.Hub.RED_CENTER.getX(), 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.FIELD_LENGTH, 0.0, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.RED_CENTER.getX(), FieldConstants.FIELD_WIDTH, 0.0, Pose3d.kZero.getRotation()),
            new Pose3d(FieldConstants.Hub.RED_CENTER.getX(), 0.0, 0.0, Pose3d.kZero.getRotation()));
    private String displayedAutoName = "";
    private boolean displayedAutoFlipped = false;
    private double[] displayedAutoArray = new double[0];
    private double[] displayedStartingPoseArray = new double[0];

    private static double[] posesToFieldObjectArray(Pose3d... poses) {
        double[] packed = new double[poses.length * 3];
        for (int i = 0; i < poses.length; i++) {
            packed[i * 3] = poses[i].getX();
            packed[i * 3 + 1] = poses[i].getY();
            packed[i * 3 + 2] = poses[i].getRotation().toRotation2d().getDegrees();
        }
        return packed;
    }

    private static double[] posesToFieldObjectArray(List<Pose2d> poses) {
        double[] packed = new double[poses.size() * 3];
        for (int i = 0; i < poses.size(); i++) {
            packed[i * 3] = poses.get(i).getX();
            packed[i * 3 + 1] = poses.get(i).getY();
            packed[i * 3 + 2] = poses.get(i).getRotation().getDegrees();
        }
        return packed;
    }

    public void setSelectedAuto(String autoName, boolean flipForAlliance) {
        String normalizedAutoName = autoName == null ? "" : autoName;
        if (!Objects.equals(displayedAutoName, normalizedAutoName) || displayedAutoFlipped != flipForAlliance) {
            displayedAutoName = normalizedAutoName;
            displayedAutoFlipped = flipForAlliance;
            displayedAutoArray = loadSelectedAutoArray(normalizedAutoName, flipForAlliance);
        }

        selectedAutoNamePub.set(displayedAutoName);
        trajectoryPub.set(displayedAutoArray);
        startingPosePub.set(displayedStartingPoseArray);
    }

    private double[] loadSelectedAutoArray(String autoName, boolean flipForAlliance) {
        if (autoName.isEmpty()) {
            displayedStartingPoseArray = new double[0];
            return new double[0];
        }

        try {
            List<Pose2d> autoPoses = new ArrayList<>();
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            if (!paths.isEmpty()) {
                PathPlannerPath firstPath = flipForAlliance ? paths.get(0).flipPath() : paths.get(0);
                Pose2d startingPose = new Pose2d(
                    firstPath.getPoint(0).position,
                    firstPath.getIdealStartingState() != null
                        ? firstPath.getIdealStartingState().rotation()
                        : Pose2d.kZero.getRotation());
                displayedStartingPoseArray = posesToFieldObjectArray(List.of(startingPose));
            } else {
                displayedStartingPoseArray = new double[0];
            }

            for (PathPlannerPath path : paths) {
                autoPoses.addAll((flipForAlliance ? path.flipPath() : path).getPathPoses());
            }
            return posesToFieldObjectArray(autoPoses);
        } catch (IOException | ParseException | RuntimeException ex) {
            DriverStation.reportError("Failed to publish selected auto '" + autoName + "'", ex.getStackTrace());
            displayedStartingPoseArray = new double[0];
            return new double[0];
        }
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);
        blueHubPub.set(m_blueHubArray);
        redHubPub.set(m_redHubArray);
        bluePassingTargetsPub.set(m_bluePassingTargetsArray);
        redPassingTargetsPub.set(m_redPassingTargetsArray);
        blueScoringZonePub.set(m_blueScoringZoneArray);
        bluePassingZonePub.set(m_bluePassingZoneArray);
        redScoringZonePub.set(m_redScoringZoneArray);
        redPassingZonePub.set(m_redPassingZoneArray);
        selectedAutoNamePub.set(displayedAutoName);
        trajectoryPub.set(displayedAutoArray);
        startingPosePub.set(displayedStartingPoseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
            SmartDashboard.putNumber(
                DRIVE_CURRENT_KEYS[i],
                drivetrain.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
        }
    }
}
