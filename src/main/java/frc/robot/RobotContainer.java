// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private static final String MANUAL_SHOOTER_ENABLE_KEY = "Tuning/Manual Shooter Mode";
    private static final String AUTO_AIM_DISABLE_KEY = "Auto Aim Disabled (On = Disable)";
    private static final String MANUAL_SHOOTER_SPEED_KEY = "Tuning/Manual Shooter RPS";
    private static final String MANUAL_HOOD_ANGLE_KEY = "Tuning/Manual Hood Angle";
    private static final String MANUAL_SHOOTER_MAX_KEY = "Tuning/Manual Shooter Max RPS";
    private static final String MANUAL_HOOD_MAX_KEY = "Tuning/Manual Hood Max Angle";
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
        SmartDashboard.putBoolean(AUTO_AIM_DISABLE_KEY, true);    // TRUE = auto aim disabled by default
        SmartDashboard.putNumber(MANUAL_SHOOTER_SPEED_KEY, 0.0);
        SmartDashboard.putNumber(MANUAL_HOOD_ANGLE_KEY, Constants.Hood.MIN_ANGLE);
        SmartDashboard.putNumber(MANUAL_SHOOTER_MAX_KEY, Constants.Shooter.MAX_SPEED_RPS);
        SmartDashboard.putNumber(MANUAL_HOOD_MAX_KEY, Constants.Hood.MAX_ANGLE);
        SmartDashboard.putBoolean(FEEDER_PULSE_ENABLED_KEY, feederPulseEnabled);
        SmartDashboard.putData("Zero Intake", Commands.runOnce(s_intake::zeroEncoder, s_intake).ignoringDisable(true));
        SmartDashboard.putData("Stow Intake", Commands.runOnce(s_intake::stow, s_intake).ignoringDisable(true));
        SmartDashboard.putData("Zero Hood", Commands.runOnce(s_hood::zeroToMinimumAngle, s_hood).ignoringDisable(true));
    }

    public void periodic()
    {
        if (!SmartDashboard.getBoolean(MANUAL_SHOOTER_ENABLE_KEY, false)) 
        {
            return;
        }
        if (!SmartDashboard.getBoolean(AUTO_AIM_DISABLE_KEY, false)) 
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

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        ); 
        
        drivetrain.registerTelemetry(logger::telemeterize);

        // Assign Driver Controls:
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));    // Reset the field-centric heading on left bumper press.
        driver.start().onTrue(Commands.runOnce(() -> s_vision.resetPoseFromVision(), s_vision));
        /*
        driver.a().onTrue(Commands.runOnce(() -> {
            double down = Constants.Intake.PIVOT_ANGLE_DOWN;
            double stowed = Constants.Intake.PIVOT_ANGLE_UP_STOWED;
            double midpoint = (down + stowed) / 2.0;
            if (s_intake.getAngle() <= midpoint) {
                s_intake.setAngle(down);
            } else {
                s_intake.setAngle(stowed);
            }
        }, s_intake));
        */

        /*
        /*
        var autoAimTrigger = driver.leftTrigger();
        autoAimTrigger.whileTrue(new AutoAim(     // Auto aims swerve, hood, and shooter speed while left trigger held.
            drivetrain,
            s_vision,
            s_hood,
            s_shooter,
            driver::getLeftY,
            driver::getLeftX,
            driver.getHID(),
            MaxSpeed,
            MaxAngularRate
        ));
        */

        driver.leftBumper().whileTrue(Commands.run(() -> {  // LONG SHOT
            s_shooter.shoot(70);    
            s_hood.setAngle(Constants.Hood.MIN_ANGLE);
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, s_shooter.isAtSpeed() ? 1.0 : 0.0);
        }, s_shooter, s_hood).finallyDo((interrupted) -> {
            s_shooter.stop();
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }));

        driver.leftTrigger().whileTrue(Commands.run(() -> { // NORMAL SHOT
            s_shooter.shoot(60);    // was 69 in match 72
            s_hood.setAngle(Constants.Hood.MIN_ANGLE);
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, s_shooter.isAtSpeed() ? 1.0 : 0.0);
        }, s_shooter, s_hood).finallyDo((interrupted) -> {
            s_shooter.stop();
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }));

                
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

        driver.rightTrigger().whileTrue(Commands.parallel(    // Run the floor and pulse/continuous feeder while right trigger held.
            Commands.run(() -> s_floor.setDumbSpeed(.5), s_floor),
            feederPulse
        ));
        driver.rightTrigger().onFalse(new ParallelCommandGroup
            (
                Commands.runOnce(() -> s_floor.stop()),
                Commands.runOnce(() -> s_feeder.stop(), s_feeder)
            )
        );

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
            boolean disabled = SmartDashboard.getBoolean(AUTO_AIM_DISABLE_KEY, false);
            SmartDashboard.putBoolean(AUTO_AIM_DISABLE_KEY, !disabled);
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
        "DeployAndIntake",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
                s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN)
            ),
            new WaitCommand(1.5),
            Commands.runOnce(() -> 
                s_intake.setIntakePercent(Constants.Intake.INTAKE_SPEED)
                )
            )
        );

        NamedCommands.registerCommand(
        "DeployIntake",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
                s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN)
                )
            )
        );

        NamedCommands.registerCommand(
        "StowAndStopIntake",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
            s_intake.setIntakePercent(0)
            ),
            new WaitCommand(0.2),
            Commands.runOnce(() -> s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_UP_STOWED)
                )
            )
        );

        NamedCommands.registerCommand(
        "AutoFeed",
        new AutoFeed(s_floor, s_feeder, 1.0)
        );

        NamedCommands.registerCommand(
        "ShootAllHopper",
        Commands.parallel(
            new AutoAim(
                drivetrain,
                s_vision,
                s_hood,
                s_shooter,
                () -> 0.0,
                () -> 0.0,
                driver.getHID(),
                MaxSpeed,
                MaxAngularRate
            ),
            Commands.sequence(
                new WaitCommand(0.5),
                new AutoFeed(s_floor, s_feeder, 5)
            )
        )
        );
        
        NamedCommands.registerCommand(
        "Shoot8",
        Commands.parallel(
            Commands.runOnce(() -> s_shooter.shoot(69)),
            Commands.sequence(
                new WaitCommand(1.5),
                new AutoFeed(s_floor, s_feeder, 3)
            )
        )
        );

        NamedCommands.registerCommand(
        "ShootALL",
        Commands.parallel(
            Commands.runOnce(() -> s_shooter.shoot(69)),
            Commands.sequence(
                new WaitCommand(1.5),
                new AutoFeed(s_floor, s_feeder, 8)
            )
        )
        );

        NamedCommands.registerCommand(
            "StopShooter",
            new SequentialCommandGroup(
                Commands.runOnce(() -> s_shooter.stop()
                )
            )
            );

        NamedCommands.registerCommand(
        "StopHopperDump",
        new SequentialCommandGroup(
            Commands.runOnce(() -> 
            s_floor.stop()
            ),
            Commands.runOnce(() -> s_feeder.setFeederSpeed(0)),
            new WaitCommand(1), 
            Commands.runOnce(() -> s_shooter.shoot(0))
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
