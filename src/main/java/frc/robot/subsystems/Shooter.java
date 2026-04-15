package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
    private static final double STATOR_CURRENT_LIMIT_AMPS = 30.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 20.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

    // Suggested starting values for VelocityTorqueCurrentFOC.
    // These gains are in torque-current units, so they are much smaller than velocity-voltage gains.
    private static final double SLOT0_KS = 3.0;
    private static final double SLOT0_KP = 2;    // was 1.45
    private static final double SLOT0_KI = 0.0;
    private static final double SLOT0_KD = 0.0;
    private static final double SLOT0_KV = 0.26;
    private static final double SLOT0_KA = 0.0;
    private static final double REQUEST_FEEDFORWARD_AMPS = 2.0;
    private static final double FOLLOWER_STATUS_UPDATE_HZ = 100.0;

    private static final double PEAK_FORWARD_TORQUE_CURRENT_AMPS = 120.0;
    private static final double PEAK_REVERSE_TORQUE_CURRENT_AMPS = -120.0;
    private static final double TORQUE_NEUTRAL_DEADBAND_AMPS = 0.0;

    private double currentSpeedSetpointRps = 0.0;

    private final TalonFX left_shooter;
    private final TalonFX right_shooter;
    private final TalonFX left_middle_shooter;
    private final TalonFX right_middle_shooter;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

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
        BaseStatusSignal.setUpdateFrequencyForAll(
                FOLLOWER_STATUS_UPDATE_HZ,
                left_shooter.getTorqueCurrent(),
                left_shooter.getVelocity());
        // set followers
        configureFollowers();
        // Initialize feeder-specific components here
    }
    
    public void shoot(double speedRPS) 
    {
        configureFollowers();
        StatusCode status = left_shooter.setControl(
                velocityRequest
                        .withVelocity(RotationsPerSecond.of(speedRPS))
                        .withFeedForward(REQUEST_FEEDFORWARD_AMPS));
        if (!status.isOK()) {
            System.out.println("Left Shooter setControl failed: " + status);
        }
        currentSpeedSetpointRps = speedRPS;
    }

    public void setShooterPercent(double percentOutput) 
    {
        double clamped = Math.max(-1.0, Math.min(1.0, percentOutput));
        shoot(clamped * Constants.Shooter.MAX_SPEED_RPS);
    }

    public void stopShooting() 
    {
        StatusCode status = left_shooter.setControl(
                velocityRequest
                        .withVelocity(RotationsPerSecond.of(0))
                        .withFeedForward(0));
        if (!status.isOK()) {
            System.out.println("Left Shooter stop setControl failed: " + status);
        }
        currentSpeedSetpointRps = 0.0;
    }

    public void stop()
    {
        left_shooter.set(0);
        currentSpeedSetpointRps = 0.0;
    }

    public void shootDumb()
    {
        left_shooter.set(1);
        currentSpeedSetpointRps = Constants.Shooter.MAX_SPEED_RPS;
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

    public double getSetpointRps()
    {
        return currentSpeedSetpointRps;
    }

    private void configureFollowers()
    {
        StatusCode rightMiddleStatus = right_middle_shooter.setControl(
                new Follower(left_shooter.getDeviceID(), MotorAlignmentValue.Aligned)
                        .withUpdateFreqHz(FOLLOWER_STATUS_UPDATE_HZ));
        StatusCode rightStatus = right_shooter.setControl(
                new Follower(left_shooter.getDeviceID(), MotorAlignmentValue.Opposed)
                        .withUpdateFreqHz(FOLLOWER_STATUS_UPDATE_HZ));
        StatusCode leftMiddleStatus = left_middle_shooter.setControl(
                new Follower(left_shooter.getDeviceID(), MotorAlignmentValue.Opposed)
                        .withUpdateFreqHz(FOLLOWER_STATUS_UPDATE_HZ));

        if (!rightMiddleStatus.isOK()) {
            System.out.println("Right Middle Shooter follower failed: " + rightMiddleStatus);
        }
        if (!rightStatus.isOK()) {
            System.out.println("Right Shooter follower failed: " + rightStatus);
        }
        if (!leftMiddleStatus.isOK()) {
            System.out.println("Left Middle Shooter follower failed: " + leftMiddleStatus);
        }
    }

    private void configureMotor(TalonFX motor, InvertedValue invertedValue, String motorName) 
    {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT0_KS)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD)
                        .withKV(SLOT0_KV)
                        .withKA(SLOT0_KA))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(PEAK_FORWARD_TORQUE_CURRENT_AMPS)
                        .withPeakReverseTorqueCurrent(PEAK_REVERSE_TORQUE_CURRENT_AMPS)
                        .withTorqueNeutralDeadband(TORQUE_NEUTRAL_DEADBAND_AMPS));
        shooterConfigs.MotorOutput.Inverted = invertedValue;
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
