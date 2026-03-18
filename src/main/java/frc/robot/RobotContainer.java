// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer 
{
    private static final String MANUAL_SHOOTER_ENABLE_KEY = "Tuning/Manual Shooter Mode";
    private static final String MANUAL_SHOOTER_SPEED_KEY = "Tuning/Manual Shooter RPS";
    private static final String MANUAL_HOOD_ANGLE_KEY = "Tuning/Manual Hood Angle";
    private static final String MANUAL_SHOOTER_MAX_KEY = "Tuning/Manual Shooter Max RPS";
    private static final String MANUAL_HOOD_MAX_KEY = "Tuning/Manual Hood Max Angle";

    // Declare and instantiate variables:
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    public RobotContainer() 
    {
        initializeTuningDashboard();
        registerNamedCommands();
        configureBindings();
    }

    private void initializeTuningDashboard()
    {
        SmartDashboard.putBoolean(MANUAL_SHOOTER_ENABLE_KEY, false);
        SmartDashboard.putNumber(MANUAL_SHOOTER_SPEED_KEY, 0.0);
        SmartDashboard.putNumber(MANUAL_HOOD_ANGLE_KEY, Constants.Hood.MIN_ANGLE);
        SmartDashboard.putNumber(MANUAL_SHOOTER_MAX_KEY, Constants.Shooter.MAX_SPEED_RPS);
        SmartDashboard.putNumber(MANUAL_HOOD_MAX_KEY, Constants.Hood.MAX_ANGLE);
        SmartDashboard.putData("Zero Intake", Commands.runOnce(s_intake::zeroEncoder, s_intake).ignoringDisable(true));
        SmartDashboard.putData("Zero Hood", Commands.runOnce(s_hood::zeroToMinimumAngle, s_hood).ignoringDisable(true));
    }

    public void periodic()
    {
        if (!SmartDashboard.getBoolean(MANUAL_SHOOTER_ENABLE_KEY, false)) 
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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        ); 
        
        drivetrain.registerTelemetry(logger::telemeterize);
            
        // Assign Test controls:
        testController.rightBumper().whileTrue(s_shooter.runOnce(() -> s_shooter.setShooterPercent(1)));    // Max is 1
        testController.rightBumper().onFalse(s_shooter.runOnce(() -> s_shooter.stop()));

        testController.leftBumper().whileTrue(s_feeder.runOnce(() -> s_feeder.setFeederPercent(1)));               // Max is 1
        testController.leftBumper().onFalse(s_feeder.runOnce(() -> s_feeder.stop()));

        testController.leftTrigger().whileTrue(s_floor.runOnce(() -> s_floor.setFloorPercent(0.5)));        // Max is 1
        testController.leftTrigger().onFalse(s_floor.runOnce(() -> s_floor.stop()));

        testController.rightTrigger().whileTrue(s_intake.runOnce(() -> s_intake.setIntakePercent(1)));      // Max is 1
        testController.rightTrigger().onFalse(s_intake.runOnce(() -> s_intake.stop()));

        testController.a().onTrue(s_intake.runOnce(() -> s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_UP_STOWED)));
        testController.y().onTrue(s_intake.runOnce(() -> s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN)));

        // Assign Driver Controls:
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));    // Reset the field-centric heading on left bumper press.

        driver.leftTrigger().whileTrue(new AutoAim(     // Auto aims swerve, hood, and shooter speed while left trigger held.
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
        
        driver.rightTrigger().whileTrue(new ParallelCommandGroup    // Run the floor and feeder to shoot balls while right trigger held.
            (
                Commands.runOnce(() -> s_floor.setFloorPercent(Constants.Floor.SHOOT_SPEED)),
                Commands.runOnce(() -> s_feeder.setFeederPercent(Constants.Feeder.SHOOT_SPEED))
            )
        );

        driver.rightBumper().toggleOnTrue(new AutoIntake(s_intake, s_floor));   // Toggle the intake and floor to intake balls when right bumper pressed.
        
        driver.leftBumper().whileTrue(Commands.startEnd(    // Run the intake, floor, and feeder in reverse to eject balls while left bumper held - for unjamming or dumping.
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
        // ...
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Command getAutonomousCommand() 
    {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
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
