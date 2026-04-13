// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoFeed;
import frc.robot.commands.AutoIntake;

import frc.robot.subsystems.*;

import frc.robot.generated.TunerConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer 
{
    private static final String MATCH_TIME_REMAINING_KEY = "Match/MatchTimeRemainingSec";
    private static final String MATCH_SCORING_PERIOD_KEY = "Match/ScoringPeriod";
    private static final String MATCH_PERIOD_TIME_REMAINING_KEY = "Match/ScoringPeriodRemainingSec";
    private static final String MATCH_ACTIVE_ALLIANCE_KEY = "Match/ActiveScoringAlliance";
    private static final String MATCH_FIRST_SCORING_ALLIANCE_KEY = "Match/FirstScoringAlliance";
    private static final String MATCH_FIRST_INACTIVE_ALLIANCE_KEY = "Match/FirstInactiveAlliance";
    private static final String MATCH_CAN_SHOOT_KEY = "Match/CanShoot";
    private static final String MATCH_BOTH_CAN_SCORE_KEY = "Match/BothAlliancesCanScore";
    private static final String MATCH_HAS_GAME_DATA_KEY = "Match/HasGameData";
    private static final String MATCH_GAME_DATA_KEY = "Match/GameSpecificMessage";
    private static final String MATCH_ALLIANCE_KEY = "Match/MyAlliance";
    private static final String MANUAL_SHOOTER_ENABLE_KEY = "Tuning/Manual Shooter Mode";
    private static final String AUTO_AIM_ENABLED_KEY = "Auto Aim Enabled";
    private static final String MANUAL_SHOOTER_SPEED_KEY = "Tuning/Manual Shooter RPS";
    private static final String MANUAL_HOOD_ANGLE_KEY = "Tuning/Manual Hood Angle";
    private static final String MANUAL_SHOOTER_MAX_KEY = "Tuning/Manual Shooter Max RPS";
    private static final String MANUAL_HOOD_MAX_KEY = "Tuning/Manual Hood Max Angle";
    private static final String SHOOTER_IDLE_AT_25_KEY = "Shooter Idle @ 25%";
    private static final String FEEDER_PULSE_ENABLED_KEY = "Feeder/Pulse Enabled";
    private static final double FEEDER_PULSE_ON_SECONDS = 0.1;
    private static final double FEEDER_PULSE_OFF_SECONDS = 0.2;

    // Declare and instantiate variables:
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static final double TRANSLATION_SLEW_RATE = 3.0; // units per second
    private static final double ROTATION_SLEW_RATE = 6.0; // units per second

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(TRANSLATION_SLEW_RATE);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(TRANSLATION_SLEW_RATE);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(ROTATION_SLEW_RATE);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController testController = new CommandXboxController(2);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake s_intake = new Intake();
    private final Hood s_hood = new Hood();
    private final Shooter s_shooter = new Shooter();
    private final Feeder s_feeder = new Feeder();
    private final Floor s_floor = new Floor();
    private final Vision s_vision = new Vision(drivetrain);
    private final SendableChooser<Command> autoChooser;
    private final Timer feederPulseTimer = new Timer();
    private boolean feederPulseEnabled = true;

    public RobotContainer() 
    {
        initializeTuningDashboard();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Right-Mid-Right-Score");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void initializeTuningDashboard()
    {
        SmartDashboard.putBoolean(MANUAL_SHOOTER_ENABLE_KEY, false);
        SmartDashboard.putBoolean(AUTO_AIM_ENABLED_KEY, true);    // TRUE = auto aim enabled by default
        SmartDashboard.putNumber(MANUAL_SHOOTER_SPEED_KEY, 0.0);
        SmartDashboard.putNumber(MANUAL_HOOD_ANGLE_KEY, Constants.Hood.MIN_ANGLE);
        SmartDashboard.putNumber(MANUAL_SHOOTER_MAX_KEY, Constants.Shooter.MAX_SPEED_RPS);
        SmartDashboard.putNumber(MANUAL_HOOD_MAX_KEY, Constants.Hood.MAX_ANGLE);
        SmartDashboard.putBoolean(SHOOTER_IDLE_AT_25_KEY, false);
        SmartDashboard.putBoolean(FEEDER_PULSE_ENABLED_KEY, feederPulseEnabled);
        SmartDashboard.putBoolean(AutoFeed.INTAKE_AGITATION_ENABLED_KEY, false);
        SmartDashboard.putBoolean(Drivetrain.TILT_BASED_VISION_UPDATES_ENABLED_KEY, true);
        SmartDashboard.putData("Zero Intake", Commands.runOnce(s_intake::zeroEncoder, s_intake).ignoringDisable(true));
        SmartDashboard.putData("Stow Intake", Commands.runOnce(s_intake::stow, s_intake).ignoringDisable(true));
        SmartDashboard.putData("Zero Hood", Commands.runOnce(s_hood::zeroToMinimumAngle, s_hood).ignoringDisable(true));
    }

    public void periodic()
    {
        updateSelectedAutoPreview();
        updateMatchTimingDashboard();

        if (!SmartDashboard.getBoolean(MANUAL_SHOOTER_ENABLE_KEY, false)) 
        {
            return;
        }
        if (SmartDashboard.getBoolean(AUTO_AIM_ENABLED_KEY, true)) 
        {
            return;
        }

        double shooterMax = SmartDashboard.getNumber(MANUAL_SHOOTER_MAX_KEY, Constants.Shooter.MAX_SPEED_RPS);
        double hoodMax = SmartDashboard.getNumber(MANUAL_HOOD_MAX_KEY, Constants.Hood.MAX_ANGLE);

        double shooterRps = MathUtil.clamp(
            SmartDashboard.getNumber(MANUAL_SHOOTER_SPEED_KEY, 0.0),
            0.0,
            shooterMax);
        double hoodAngle = MathUtil.clamp(
            SmartDashboard.getNumber(MANUAL_HOOD_ANGLE_KEY, 0.0),
            Constants.Hood.MIN_ANGLE,
            hoodMax);

        s_shooter.shoot(shooterRps);
        s_hood.setAngle(hoodAngle);

        SmartDashboard.putNumber("Tuning/Applied Shooter RPS", shooterRps);
        SmartDashboard.putNumber("Tuning/Applied Hood Angle", hoodAngle);
    }

    private void updateSelectedAutoPreview()
    {
        Command selectedAuto = autoChooser.getSelected();
        String selectedAutoName = selectedAuto instanceof PathPlannerAuto pathPlannerAuto
            ? pathPlannerAuto.getName()
            : "";
        boolean flipForAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

        logger.setSelectedAuto(selectedAutoName, flipForAlliance);
    }

    private void updateMatchTimingDashboard()
    {
        double matchTimeRemainingSec = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance();
        MatchTimingStatus status = getMatchTimingStatus(matchTimeRemainingSec, alliance, gameData);

        SmartDashboard.putNumber(MATCH_TIME_REMAINING_KEY, matchTimeRemainingSec);
        SmartDashboard.putString(MATCH_SCORING_PERIOD_KEY, status.scoringPeriod());
        SmartDashboard.putNumber(MATCH_PERIOD_TIME_REMAINING_KEY, status.periodTimeRemainingSec());
        SmartDashboard.putString(MATCH_ACTIVE_ALLIANCE_KEY, status.activeScoringAlliance());
        SmartDashboard.putString(MATCH_FIRST_SCORING_ALLIANCE_KEY, status.firstScoringAlliance());
        SmartDashboard.putString(MATCH_FIRST_INACTIVE_ALLIANCE_KEY, status.firstInactiveAlliance());
        SmartDashboard.putBoolean(MATCH_CAN_SHOOT_KEY, status.canShoot());
        SmartDashboard.putBoolean(MATCH_BOTH_CAN_SCORE_KEY, status.bothAlliancesCanScore());
        SmartDashboard.putBoolean(MATCH_HAS_GAME_DATA_KEY, !gameData.isEmpty());
        SmartDashboard.putString(MATCH_GAME_DATA_KEY, gameData.isEmpty() ? "None" : gameData);
        SmartDashboard.putString(MATCH_ALLIANCE_KEY, alliance.map(Enum::name).orElse("Unknown"));
    }

    private MatchTimingStatus getMatchTimingStatus(
        double matchTimeRemainingSec,
        java.util.Optional<DriverStation.Alliance> alliance,
        String gameData)
    {
        if (DriverStation.isAutonomousEnabled()) {
            return new MatchTimingStatus(
                "Auto",
                Math.max(0.0, matchTimeRemainingSec),
                "Both",
                "Unknown",
                "Unknown",
                true,
                true);
        }

        if (DriverStation.isTeleopEnabled()) {
            return getTeleopTimingStatus(matchTimeRemainingSec, alliance, gameData);
        }

        return new MatchTimingStatus(
            "Disabled",
            0.0,
            "None",
            "Unknown",
            "Unknown",
            false,
            false);
    }

    private MatchTimingStatus getTeleopTimingStatus(
        double matchTimeRemainingSec,
        java.util.Optional<DriverStation.Alliance> alliance,
        String gameData)
    {
        if (matchTimeRemainingSec > 130.0) {
            return new MatchTimingStatus(
                "Transition Shift",
                matchTimeRemainingSec - 130.0,
                "Both",
                getFirstScoringAlliance(gameData),
                getFirstInactiveAlliance(gameData),
                true,
                true);
        }

        if (matchTimeRemainingSec > 105.0) {
            return buildAllianceShiftStatus("Shift 1", matchTimeRemainingSec - 105.0, true, alliance, gameData);
        }

        if (matchTimeRemainingSec > 80.0) {
            return buildAllianceShiftStatus("Shift 2", matchTimeRemainingSec - 80.0, false, alliance, gameData);
        }

        if (matchTimeRemainingSec > 55.0) {
            return buildAllianceShiftStatus("Shift 3", matchTimeRemainingSec - 55.0, true, alliance, gameData);
        }

        if (matchTimeRemainingSec > 30.0) {
            return buildAllianceShiftStatus("Shift 4", matchTimeRemainingSec - 30.0, false, alliance, gameData);
        }

        return new MatchTimingStatus(
            "End Game",
            Math.max(0.0, matchTimeRemainingSec),
            "Both",
            getFirstScoringAlliance(gameData),
            getFirstInactiveAlliance(gameData),
            true,
            true);
    }

    private MatchTimingStatus buildAllianceShiftStatus(
        String shiftName,
        double periodTimeRemainingSec,
        boolean firstScoringAllianceActive,
        java.util.Optional<DriverStation.Alliance> alliance,
        String gameData)
    {
        String firstScoringAlliance = getFirstScoringAlliance(gameData);
        String firstInactiveAlliance = getFirstInactiveAlliance(gameData);
        String activeAlliance = firstScoringAllianceActive
            ? firstScoringAlliance
            : oppositeAllianceName(firstScoringAlliance);
        boolean canShoot = alliance.isPresent()
            && activeAlliance.equalsIgnoreCase(alliance.get().name());

        if (activeAlliance.equals("Unknown")) {
            canShoot = false;
        }

        return new MatchTimingStatus(
            shiftName,
            periodTimeRemainingSec,
            activeAlliance,
            firstScoringAlliance,
            firstInactiveAlliance,
            canShoot,
            false);
    }

    private String getFirstInactiveAlliance(String gameData)
    {
        return switch (gameData) {
            case "R" -> "Red";
            case "B" -> "Blue";
            default -> "Unknown";
        };
    }

    private String getFirstScoringAlliance(String gameData)
    {
        return switch (gameData) {
            case "R" -> "Blue";
            case "B" -> "Red";
            default -> "Unknown";
        };
    }

    private String oppositeAllianceName(String allianceName)
    {
        return switch (allianceName) {
            case "Red" -> "Blue";
            case "Blue" -> "Red";
            default -> "Unknown";
        };
    }

    private record MatchTimingStatus(
        String scoringPeriod,
        double periodTimeRemainingSec,
        String activeScoringAlliance,
        String firstScoringAlliance,
        String firstInactiveAlliance,
        boolean canShoot,
        boolean bothAlliancesCanScore) {
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void configureBindings() 
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xLimiter.calculate(-driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(yLimiter.calculate(-driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotLimiter.calculate(-driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        s_shooter.setDefaultCommand(
            Commands.run(() -> 
            {
                boolean manualShooterEnabled = SmartDashboard.getBoolean(MANUAL_SHOOTER_ENABLE_KEY, false);     // Tuning mode
                boolean shooterIdleAt25Enabled = SmartDashboard.getBoolean(SHOOTER_IDLE_AT_25_KEY, true);      // Elastic Switch

                if (shooterIdleAt25Enabled && !manualShooterEnabled) 
                {
                    s_shooter.setShooterPercent(0.25);
                } 
                else 
                {
                    s_shooter.stopShooting();
                }
            }, s_shooter)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        ); 
        
        drivetrain.registerTelemetry(logger::telemeterize);

        /*
        Command feederPulse = Commands.run(() -> {
            feederPulseEnabled = SmartDashboard.getBoolean(FEEDER_PULSE_ENABLED_KEY, feederPulseEnabled);
            if (!feederPulseEnabled) {
                s_feeder.setFeederPercent(1);
                return;
            }

            double t = feederPulseTimer.get();
            double cycleSeconds = FEEDER_PULSE_ON_SECONDS + FEEDER_PULSE_OFF_SECONDS;
            if (t >= cycleSeconds) {
                feederPulseTimer.reset();
                t = 0.0;
            }

            if (t < FEEDER_PULSE_ON_SECONDS) {
                s_feeder.setFeederPercent(.5);
            } else {
                s_feeder.stop();
            }
        }, s_feeder).beforeStarting(() -> {
            feederPulseTimer.reset();
            feederPulseTimer.start();
        }).finallyDo((interrupted) -> {
            feederPulseTimer.stop();
            s_feeder.stop();
        });
        */

        // Assign Driver Controls:
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));    // Reset the field-centric heading on left bumper press.
        driver.start().onTrue(Commands.runOnce(() -> s_vision.resetPoseFromVision(), s_vision));
        driver.y().onTrue(Commands.runOnce(() -> {
            double down = Constants.Intake.PIVOT_ANGLE_DOWN;
            double stowed = Constants.Intake.PIVOT_ANGLE_UP_STOWED;
            double midpoint = (down + stowed) / 2.0;
            if (s_intake.getAngle() <= midpoint) {
                s_intake.setAngle(down);
            } else {
                s_intake.setAngle(stowed);
            }
        }, s_intake)); 
        driver.leftBumper().whileTrue(Commands.run(() -> {  // LONG SHOT
            s_shooter.shoot(70);    
            s_hood.setAngle(Constants.Hood.MIN_ANGLE);
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, s_shooter.isAtSpeed() ? 1.0 : 0.0);
        }, s_shooter, s_hood).finallyDo((interrupted) -> {
            s_shooter.stop();
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }));

        /*
        driver.leftTrigger().whileTrue(Commands.run(() -> { // NORMAL SHOT
            s_shooter.shoot(Constants.Shooter.SHOOTER_SPEED_CLOSE);    // was 69 in match 72
            s_hood.setAngle(Constants.Hood.MIN_ANGLE);
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, s_shooter.isAtSpeed() ? 1.0 : 0.0);
        }, s_shooter, s_hood).finallyDo((interrupted) -> {
            s_shooter.stop();
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }));
        */

        driver.leftTrigger().whileTrue(new AutoAim(
            drivetrain,
            s_vision,
            s_hood,
            s_shooter,
            driver::getLeftY,
            driver::getLeftX,
            driver.getHID(),
            MaxSpeed,
            MaxAngularRate
        )); // Auto aim while left trigger held.

        driver.rightTrigger().whileTrue(Commands.parallel(    // Run the floor and pulse/continuous feeder while right trigger held.
            //Commands.run(() -> s_floor.setDumbSpeed(.5), s_floor)  // TODO - try setfloorpercent instead of setdumb
            new AutoFeed(s_intake, s_floor, s_feeder)
        ));
        driver.rightTrigger().onFalse(new ParallelCommandGroup(
                Commands.runOnce(() -> s_floor.stop()),
                Commands.runOnce(() -> s_feeder.stop(), s_feeder)
        ));

        driver.rightBumper().toggleOnTrue(new AutoIntake(s_intake, s_floor, s_feeder));   // Toggle the intake and floor to intake balls when right bumper pressed.
        
        driver.a().whileTrue(Commands.startEnd(    // Run the intake, floor, and feeder in reverse to eject balls while left bumper held - for unjamming or dumping.
            () -> {
                s_intake.setIntakePercent(-Constants.Intake.INTAKE_SPEED);
                s_floor.setFloorPercent(-Constants.Floor.FLOOR_SPEED);
                s_feeder.setFeederPercent(-Constants.Feeder.FEEDER_SPEED);
            },
            () -> {
                s_intake.stop();
                s_floor.stop();
                s_feeder.stop();
            },
            s_intake,
            s_floor,
            s_feeder
        ));
        
        
        // Assign Operator Controls:
        operator.a().onTrue(Commands.runOnce(() -> {
            feederPulseEnabled = !feederPulseEnabled;
            SmartDashboard.putBoolean(FEEDER_PULSE_ENABLED_KEY, feederPulseEnabled);
        }));

        operator.y().onTrue(Commands.runOnce(() -> {
            boolean enabled = SmartDashboard.getBoolean(AUTO_AIM_ENABLED_KEY, true);
            SmartDashboard.putBoolean(AUTO_AIM_ENABLED_KEY, !enabled);
        }));
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Command getAutonomousCommand() 
    {
        Command selectedAuto = autoChooser.getSelected();
        return selectedAuto != null ? selectedAuto : Commands.none();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Registers all named commands for the robot.
    public void registerNamedCommands()
    {
        NamedCommands.registerCommand(
        "DeployIntake",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
                s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN)
                )
            )
        );

        NamedCommands.registerCommand(
        "DeployIntakeAndStartIntaking",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
                s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN)
            ),
            Commands.runOnce(() -> 
                s_intake.setIntakePercent(Constants.Intake.INTAKE_SPEED)
                )
            )
        );

        NamedCommands.registerCommand(
        "StopIntaking",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
                s_intake.stop()
                )
            )
        );

        NamedCommands.registerCommand(
        "ShootPreloadFromStaticShooter",
        Commands.parallel(
            Commands.runOnce(() -> s_shooter.shoot(Constants.Shooter.SHOOTER_SPEED_CLOSE)),
            Commands.sequence(
                new WaitCommand(1.5),                       // TODO - Adjust shooter startup time
                new AutoFeed(s_intake, s_floor, s_feeder).withTimeout(3)  // TODO - Adjust period that shooter shoots for
            )
        )
        );

        NamedCommands.registerCommand(
        "ShootFullHopper",
        Commands.parallel(
            Commands.runOnce(() -> s_shooter.shoot(Constants.Shooter.SHOOTER_SPEED_CLOSE)),
            Commands.sequence(
                new AutoAim(drivetrain, s_vision, s_hood, s_shooter, null, null, null, MaxSpeed, MaxAngularRate),
                new WaitCommand(0), // TODO Adjust this to give the shooter more time to startup, and autoaim more time to settle. May not be necessary to add any delay here, or it may need to be increased.
                new AutoFeed(s_intake, s_floor, s_feeder).withTimeout(5)  // TODO - Adjust period that shooter shoots for
            )
        )
        );
        
        NamedCommands.registerCommand(
        "SpoolShooterToShootingSpeed",
        new SequentialCommandGroup(
            Commands.runOnce(() -> s_shooter.shoot(Constants.Shooter.SHOOTER_SPEED_CLOSE)) // TODO - Adjust shooter speed if needed
            )
        );

        NamedCommands.registerCommand(
        "StopShooter",
        new SequentialCommandGroup(
            Commands.runOnce(() -> s_shooter.stop())
            )
        );
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////
    public void printDiagnostics()
    {
        s_shooter.printDiagnostics();
        s_hood.printDiagnostics();
        s_intake.printDiagnostics();
        s_feeder.printDiagnostics();
        s_floor.printDiagnostics();
    }
}
