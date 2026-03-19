package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase 
{
    private static final double STATOR_CURRENT_LIMIT_AMPS = 120;   // 120
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;   // 60
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;

    // Initial velocity-loop tuning for 1:1 Kraken X60 shooter.
    private static final double SLOT0_KS = 0.2; // TODO - tune
    private static final double SLOT0_KA = 0.0; // TODO - tune
    // 12V / 100 RPS = 0.12 V per RPS (approx for Kraken X60 at 1:1)
    private static final double SLOT0_KV = 0.12; // TODO - tune
    private static final double SLOT0_KP = 0.1; // TODO - tune
    private static final double SLOT0_KI = 0.0; // TODO - tune
    private static final double SLOT0_KD = 0.0; // TODO - tune

    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private double currentSpeedSetpointRps = 0.0;

    private final TalonFX left_shooter;
    private final TalonFX right_shooter;
    private final TalonFX left_middle_shooter;
    private final TalonFX right_middle_shooter;
    private final VelocityVoltage  velocityRequest = new VelocityVoltage(0);

    public Shooter() 
    {
        // declare all motors
        left_shooter = new TalonFX(Constants.CAN_ID.LEFT_SHOOTER, Constants.CAN_BUS.CANIVORE);
        right_shooter = new TalonFX(Constants.CAN_ID.RIGHT_SHOOTER, Constants.CAN_BUS.CANIVORE);
        left_middle_shooter = new TalonFX(Constants.CAN_ID.LEFT_MIDDLE_SHOOTER, Constants.CAN_BUS.CANIVORE);
        right_middle_shooter = new TalonFX(Constants.CAN_ID.RIGHT_MIDDLE_SHOOTER, Constants.CAN_BUS.CANIVORE);
        // configure motors
        configureMotor(right_middle_shooter, InvertedValue.Clockwise_Positive, "Right Middle Shooter");
        configureMotor(right_shooter, InvertedValue.CounterClockwise_Positive, "Right Shooter");
        configureMotor(left_shooter, InvertedValue.Clockwise_Positive, "Left Shooter");
        configureMotor(left_middle_shooter, InvertedValue.CounterClockwise_Positive, "Left Middle Shooter");
        // set followers
        configureFollowers();
        // Initialize feeder-specific components here
    }
    
    public void shoot(double speedRPS) 
    {
        configureFollowers();
        left_shooter.setControl(velocityRequest.withVelocity(RotationsPerSecond.of(speedRPS)));
        currentSpeedSetpointRps = speedRPS;
    }

    public void setShooterPercent(double percentOutput) 
    {
        double clamped = Math.max(-1.0, Math.min(1.0, percentOutput));
        shoot(clamped * Constants.Shooter.MAX_SPEED_RPS);
    }

    public void stopShooting() 
    {
        left_shooter.setControl(velocityRequest.withVelocity(RotationsPerSecond.of(0)));
        currentSpeedSetpointRps = 0.0;
    }

    public void stop()
    {
        left_shooter.set(0);
    }

    public void shootDumb()
    {
        left_shooter.set(1);
    }

    public double getSpeed() 
    {
        return (left_shooter.getVelocity().getValueAsDouble()
            + left_middle_shooter.getVelocity().getValueAsDouble()
                + right_shooter.getVelocity().getValueAsDouble()
                    + right_middle_shooter.getVelocity().getValueAsDouble()) / 4.0;
    }
    public boolean isAtSpeed() 
    {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= Constants.Shooter.SPEED_TOLERANCE_RPS;
    }

    private void configureFollowers()
    {
        right_middle_shooter.setControl(new Follower(left_shooter.getDeviceID(), MotorAlignmentValue.Aligned));
        right_shooter.setControl(new Follower(left_shooter.getDeviceID(), MotorAlignmentValue.Opposed));
        left_middle_shooter.setControl(new Follower(left_shooter.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureMotor(TalonFX motor, InvertedValue invertedValue, String motorName) 
    {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT0_KS)
                        .withKA(SLOT0_KA)
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));
        shooterConfigs.MotorOutput.Inverted = invertedValue;
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < Constants.CONFIG_RETRIES; ++i) {
            status = motor.getConfigurator().apply(shooterConfigs);
            if (status.isOK()) 
            {
                break;
            }
        }

        if (!status.isOK()) 
        {
            System.out.println("Could not apply configs for " + motorName + ", error code: " + status);
        }
    }
    
    public void printDiagnostics() 
    {
        SmartDashboard.putNumber("Shooter Current Speed RPS", getSpeed());
        SmartDashboard.putNumber("Shooter Speed Setpoint RPS", currentSpeedSetpointRps);
        SmartDashboard.putBoolean("Shooter Is At Speed", isAtSpeed());
        SmartDashboard.putNumber("Shooter Current", left_shooter.getSupplyCurrent().getValueAsDouble()
            + left_middle_shooter.getSupplyCurrent().getValueAsDouble()
                + right_shooter.getSupplyCurrent().getValueAsDouble()
                    + right_middle_shooter.getSupplyCurrent().getValueAsDouble());
    }
}
