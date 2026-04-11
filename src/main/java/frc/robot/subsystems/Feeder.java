package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase 
{
    private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
    private static final double MAX_FEEDER_SPEED_RPS = 100.0; // 100 is max rps for Kraken X60
    private static final double SLOT0_KS = 0;
    private static final double SLOT0_KV = 0;
    private static final double SLOT0_KP = 1;
    private static final double SLOT0_KI = 0;
    private static final double SLOT0_KD = 0;

    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private double currentSpeedSetpointRps = 0.0;

    //private static final double MAX_ELEVATOR_SPEED_RPS = Constants.Shooter.MAX_ELEVATOR_SPEED_RPS;
    private final TalonFX left_feeder;
    private final TalonFX right_feeder;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private void ensureFollower() {
        right_feeder.setControl(new Follower(left_feeder.getDeviceID(), MotorAlignmentValue.Opposed));
    }
    
    public Feeder() 
    {
        left_feeder = new TalonFX(Constants.CAN_ID.LEFT_FEEDER, Constants.CAN_BUS.CANIVORE);
        right_feeder = new TalonFX(Constants.CAN_ID.RIGHT_FEEDER, Constants.CAN_BUS.CANIVORE);
        configureMotor(left_feeder, InvertedValue.CounterClockwise_Positive, "Left Feeder");
        configureMotor(right_feeder, InvertedValue.Clockwise_Positive, "Right Feeder");
        ensureFollower();

        // Initialize feeder-specific components here
    }

    public void setFeederSpeed(double speedRPS) 
    {
        ensureFollower();
        left_feeder.setControl(velocityRequest.withVelocity(RotationsPerSecond.of(speedRPS)));
        currentSpeedSetpointRps = speedRPS;
    }

    public void setFeederPercent(double percentOutput) 
    {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        setFeederSpeed(clamped * MAX_FEEDER_SPEED_RPS);
    }

    public void setDumbSpeed(double percent)
    {
        ensureFollower();
        left_feeder.set(percent);
    }

    public double getSpeed() 
    {
        return (left_feeder.getVelocity().getValueAsDouble()
            + right_feeder.getVelocity().getValueAsDouble()) / 2.0;
    }

    public boolean isAtSpeed() 
    {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= Constants.Feeder.SPEED_TOLERANCE_RPS;
    }

    public void stop() 
    {
        left_feeder.set(0);
        ensureFollower();
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
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));

        shooterConfigs.MotorOutput.Inverted = invertedValue;
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < Constants.CONFIG_RETRIES; ++i) 
        {
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
        SmartDashboard.putNumber("Feeder Current Speed RPS", getSpeed());
        SmartDashboard.putNumber("Feeder Speed Setpoint RPS", currentSpeedSetpointRps);
        SmartDashboard.putBoolean("Feeder Is At Speed", isAtSpeed());
        SmartDashboard.putNumber("Feeder Current", (left_feeder.getSupplyCurrent().getValueAsDouble()
            + right_feeder.getSupplyCurrent().getValueAsDouble())/2.0);
    }
    
}
