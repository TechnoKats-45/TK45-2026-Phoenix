// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    // Declare and instantiate variables:
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake s_intake = new Intake();
    private final Shooter s_shooter = new Shooter();
    private final Feeder s_feeder = new Feeder();
    private final Floor s_floor = new Floor();

    public RobotContainer() {
        registerNamedCommands();
        configureBindings();
    }

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
            s_intake.runFeed(Constants.Intake.INTAKE_SPEED)
        )
    )
);
    NamedCommands.registerCommand(
    "StowAndIntake",
    new SequentialCommandGroup(
        Commands.runOnce(() -> 
        s_intake.runFeed(0)
        ),
        new WaitCommand(0.2),
        Commands.runOnce(() -> s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_UP_STOWED)
            
        )
    )
);
    NamedCommands.registerCommand(
    "DumpHopper",
    new SequentialCommandGroup(
        Commands.runOnce(() -> 
        s_floor.runFeed(50)
        ),
        Commands.runOnce(() -> s_feeder.setFeederSpeed(50)),
        new WaitCommand(1), 
        Commands.runOnce(() -> s_shooter.shoot(50))
        )
    );
    NamedCommands.registerCommand(
    "StopHopperDump",
    new SequentialCommandGroup(
        Commands.runOnce(() -> 
        s_floor.runFeed(0)
        ),
        Commands.runOnce(() -> s_feeder.setFeederSpeed(0)),
        new WaitCommand(1), 
        Commands.runOnce(() -> s_shooter.shoot(0))
        )
    );
    }

    private void configureBindings() {
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

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
            //make controls:
        driver.rightBumper().onTrue(s_shooter.runOnce(() -> s_shooter.shoot(0.5)));
        driver.leftBumper().onTrue(s_feeder.runOnce(() -> s_feeder.setFeederSpeed(0.5)));
        driver.leftTrigger().onTrue(s_floor.runOnce(() -> s_floor.runFeed(0.5)));
        driver.rightTrigger().onTrue(s_intake.runOnce(() -> s_intake.runFeed(0.5)));
        //off toggles:
        driver.povRight().onTrue(s_shooter.runOnce(() -> s_shooter.shoot(0.0)));
        driver.povLeft().onTrue(s_feeder.runOnce(() -> s_feeder.setFeederSpeed(0.0)));
        driver.povDown().onTrue(s_floor.runOnce(() -> s_floor.runFeed(0.0)));
        driver.povUp().onTrue(s_intake.runOnce(() -> s_intake.runFeed(0.0)));
        //Pivot:
        driver.a().onTrue(s_intake.runOnce(() -> s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN)));
        driver.y().onTrue(s_intake.runOnce(() -> s_intake.setAngle(Constants.Intake.PIVOT_ANGLE_UP_STOWED)));
    }

    public Command getAutonomousCommand() {
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

}
