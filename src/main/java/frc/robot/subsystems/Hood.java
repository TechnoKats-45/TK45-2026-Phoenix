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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase 
{
    private static final int CONFIG_RETRIES = Constants.CONFIG_RETRIES;

    //VARS
    private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double MM_CRUISE_RPS = 1; // TODO TUNE
    private static final double MM_ACCEL_RPS2 = 10; // TODO TUNE
    private static final double SENSOR_TO_MECHANISM_RATIO = 2.0; // TODO TUNE
    private static final double PEAK_FORWARD_VOLTS= 16.0;
    private static final double PEAK_REVERSE_VOLTS= -16.0; 
    private double currentAngleSetPoint = 0.0; 
    
    //PID
    private static final double SLOT0_KS = 0.0; // TODO TUNE
    private static final double SLOT0_KV = 0.0; // TODO TUNE
    private static final double SLOT0_KP = 1.0; // TODO TUNE
    private static final double SLOT0_KI = 0.0; // TODO TUNE
    private static final double SLOT0_KD = 1.0; // TODO TUNE

    private final TalonFX hood_motor; 
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private static final InvertedValue HOOD_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

    public Hood() 
    {
        hood_motor = new TalonFX(Constants.CAN_ID.HOOD,Constants.CAN_BUS.CANIVORE);
        configureMotor(hood_motor, HOOD_MOTOR_INVERTED, "Hood Motor");
    }

    public void zeroEncoder()
    {
        hood_motor.setPosition(0.0);
    }
    public void setAngle(double angle)
    {
        currentAngleSetPoint = angle;
        hood_motor.setControl(motionMagicVoltage.withPosition(angle));
        SmartDashboard.putNumber("Hood Set Point", angle);
    }

    public double getAngle()
    {
        return hood_motor.getPosition().getValueAsDouble();
    }

    public boolean isAligned()
    {
        return Math.abs(getAngle() - currentAngleSetPoint) <= Constants.Hood.ANGLE_TOLERANCE_DEGREES;
    
    }

    private void configureMotor(TalonFX motor, InvertedValue invertedValue, String motorName) {
        TalonFXConfiguration floorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2)))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT0_KS)
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));

        floorConfigs.MotorOutput.Inverted = invertedValue;
        floorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        floorConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = motor.getConfigurator().apply(floorConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for " + motorName + ", error code: " + status);
        }
    }

    public void printDiagnostics() {
        SmartDashboard.putNumber("Hood Current Angle", getAngle());
        SmartDashboard.putNumber("Hood Angle Setpoint", currentAngleSetPoint);
        SmartDashboard.putBoolean("Hood Is Aligned", isAligned());
        SmartDashboard.putNumber("Hood Current", hood_motor.getSupplyCurrent().getValueAsDouble());
    }
}
