package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoAim extends Command 
{
        // Auto-aim rotation PID (local to AutoAim, not in Constants)
        private static final double ROTATION_KP = 20;           // 10 too low, but works, 50 TOO high, 25 slightly too high
        private static final double ROTATION_KI = 0.0;
        private static final double ROTATION_KD = 0.0;

        private final Drivetrain drivetrain;
        private final Vision vision;
        private final Hood hood;
        private final Shooter shooter;
        private final DoubleSupplier forwardSupplier;
        private final DoubleSupplier strafeSupplier;
        private final GenericHID rumbleController;
        private final double maxSpeedMetersPerSecond;
        private final double maxAngularRateRadiansPerSecond;
        private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final PIDController rotationController = new PIDController(
                ROTATION_KP,
                ROTATION_KI,
                ROTATION_KD);

        public AutoAim(
                Drivetrain drivetrain,
                Vision vision,
                Hood hood,
                Shooter shooter,
                DoubleSupplier forwardSupplier,
                DoubleSupplier strafeSupplier,
                GenericHID rumbleController,
                double maxSpeedMetersPerSecond,
                double maxAngularRateRadiansPerSecond) 
                {
                        this.drivetrain = drivetrain;
                        this.vision = vision;
                        this.hood = hood;
                        this.shooter = shooter;
                        this.forwardSupplier = forwardSupplier;
                        this.strafeSupplier = strafeSupplier;
                        this.rumbleController = rumbleController;
                        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
                        this.maxAngularRateRadiansPerSecond = maxAngularRateRadiansPerSecond;

                        rotationController.enableContinuousInput(-Math.PI, Math.PI);
                        rotationController.setTolerance(
                                Math.toRadians(Constants.Vision.AUTO_AIM_ROTATION_TOLERANCE_DEG));

                        addRequirements(drivetrain, hood, shooter);
                }

        @Override
        public void initialize() 
        {
                drivetrain.resetAutoAimHeadingOffset();
        }

        @Override
        public void execute() 
        {
                boolean testShootMode = SmartDashboard.getBoolean("Test Shooter Mode", false);
                Optional<Alliance> alliance = DriverStation.getAlliance();
                Alliance resolvedAlliance = alliance.orElse(Alliance.Blue);
                Pose3d hubCenter = FieldConstants.Hub.getCenterForAlliance(Optional.of(resolvedAlliance));
                boolean inPassingZone = FieldConstants.ShotZones.isInPassingZone(
                        drivetrain.getState().Pose,
                        Optional.of(resolvedAlliance));
                Pose3d targetPose = inPassingZone
                        ? FieldConstants.Passing.getClosestForAlliance(
                                drivetrain.getState().Pose,
                                Optional.of(resolvedAlliance))
                        : hubCenter;

                var rawHubObservation = vision.getHubObservation(Optional.of(resolvedAlliance));
                boolean canUseHubVision = vision.hasConfiguredHubTagIds(Optional.of(resolvedAlliance));
                Optional<Vision.HubObservation> hubObservation =
                        !inPassingZone && canUseHubVision ? rawHubObservation : Optional.empty();

                Translation2d robotToGoal = targetPose.toPose2d().getTranslation()
                        .minus(drivetrain.getState().Pose.getTranslation());
                Rotation2d desiredHeading = robotToGoal.getAngle()
                        .plus(Rotation2d.k180deg)
                        .plus(drivetrain.getAutoAimHeadingOffset());
                if (!testShootMode) {
                        drivetrain.setAutoAimTargetHeading(
                                desiredHeading,
                                Constants.Vision.AUTO_AIM_ROTATION_TOLERANCE_DEG);
                }

                double rangeMeters = hubObservation
                                .map(Vision.HubObservation::rangeToHubCenterMeters)
                                .orElseGet(() -> drivetrain.getState().Pose.getTranslation()
                                        .getDistance(targetPose.toPose2d().getTranslation()));
                double rangeInches = Units.metersToInches(rangeMeters);
                var shotProfile = Constants.Shooter.getShotProfileForDistanceInches(rangeInches);

                hood.setAngle(shotProfile.hoodDeg());
                shooter.shoot(shotProfile.speedRps());

                double currentHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
                double desiredHeadingRad = desiredHeading.getRadians();
                double maxAutoAimRotationRate = Math.max(
                                maxAngularRateRadiansPerSecond,
                                Units.rotationsToRadians(Constants.Vision.AUTO_AIM_MAX_ANGULAR_RATE_RPS))
                                * Constants.Vision.AUTO_AIM_MAX_ROTATION_RATE_SCALE;
                double rawRotationRate = rotationController.calculate(currentHeadingRad, desiredHeadingRad);
                double rotationRate = testShootMode
                                ? 0.0
                                : MathUtil.clamp(rawRotationRate, -maxAutoAimRotationRate, maxAutoAimRotationRate);
                double headingErrorDeg = Math.toDegrees(
                                MathUtil.angleModulus(desiredHeadingRad - currentHeadingRad));

                double forwardInput = forwardSupplier != null ? forwardSupplier.getAsDouble() : 0.0;
                double strafeInput = strafeSupplier != null ? strafeSupplier.getAsDouble() : 0.0;
                double manualRotInput = rumbleController != null ? rumbleController.getRawAxis(4) : 0.0;
                double forward = MathUtil.applyDeadband(
                        -forwardInput,
                        Constants.Vision.AUTO_AIM_TRANSLATION_DEADBAND);
                double strafe = MathUtil.applyDeadband(
                        -strafeInput,
                        Constants.Vision.AUTO_AIM_TRANSLATION_DEADBAND);
                double manualRot = MathUtil.applyDeadband(
                        -manualRotInput,
                        Constants.Vision.AUTO_AIM_TRANSLATION_DEADBAND);

                drivetrain.setControl(
                        driveRequest
                                .withDeadband(maxSpeedMetersPerSecond * Constants.Vision.AUTO_AIM_TRANSLATION_DEADBAND)
                                .withRotationalDeadband(0.0)
                                .withVelocityX(forward * maxSpeedMetersPerSecond)
                                .withVelocityY(strafe * maxSpeedMetersPerSecond)
                                .withRotationalRate(testShootMode
                                        ? manualRot * maxAngularRateRadiansPerSecond
                                        : rotationRate));

                SmartDashboard.putString("AutoAim/Alliance", resolvedAlliance.name());
                SmartDashboard.putString("AutoAim/Zone", inPassingZone ? "Passing" : "Scoring");
                SmartDashboard.putBoolean("AutoAim/UsingVision", hubObservation.isPresent());
                SmartDashboard.putBoolean("AutoAim/HubTagIdsConfigured", canUseHubVision);
                SmartDashboard.putString("AutoAim/AimSource", hubObservation.map(Vision.HubObservation::source).orElse("pose"));
                SmartDashboard.putBoolean("AutoAim/TestShootMode", testShootMode);
                SmartDashboard.putNumber("AutoAim/TargetX", targetPose.getX());
                SmartDashboard.putNumber("AutoAim/TargetY", targetPose.getY());
                SmartDashboard.putNumber("AutoAim/RangeInches", rangeInches);
                SmartDashboard.putNumber("AutoAim/DesiredHeadingDeg", desiredHeading.getDegrees());
                SmartDashboard.putNumber("AutoAim/HeadingOffsetDeg", drivetrain.getAutoAimHeadingOffsetDeg());
                SmartDashboard.putNumber("AutoAim/HeadingErrorDeg", headingErrorDeg);
                SmartDashboard.putNumber("AutoAim/HoodDeg", shotProfile.hoodDeg());
                SmartDashboard.putNumber("AutoAim/ShooterRps", shotProfile.speedRps());
                SmartDashboard.putNumber("AutoAim/RotationRateCmdRadPerSec", rotationRate);
                SmartDashboard.putBoolean(
                        "AutoAim/RotationAligned",
                        drivetrain.isRotAligned());
                SmartDashboard.putNumber(
                        "AutoAim/RobotHeadingDeg",
                        drivetrain.getState().Pose.getRotation().getDegrees());
                SmartDashboard.putNumber(
                        "AutoAim/GyroYawDeg",
                        drivetrain.getGyroYawDeg());

                boolean readyToShoot = testShootMode
                        ? (hood.isAligned() && shooter.isAtSpeed())
                        : (drivetrain.isRotAligned() && hood.isAligned() && shooter.isAtSpeed());
                if (rumbleController != null) {
                        rumbleController.setRumble(GenericHID.RumbleType.kBothRumble, readyToShoot ? 1.0 : 0.0);
                }
        }

        @Override
        public void end(boolean interrupted) 
        {

                drivetrain.clearAutoAimTargetHeading();
                drivetrain.resetAutoAimHeadingOffset();
                if (rumbleController != null) {
                        rumbleController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                }
                shooter.stop();
                rotationController.reset();
                drivetrain.setControl(
                        driveRequest
                                .withVelocityX(0.0 * maxSpeedMetersPerSecond)
                                .withVelocityY(0.0 * maxSpeedMetersPerSecond)
                                .withRotationalRate(0.0 * maxAngularRateRadiansPerSecond));
        }

        @Override
        public boolean isFinished() 
        {
                return false;
        }
}
