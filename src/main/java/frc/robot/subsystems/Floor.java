package frc.robot.subsystems;
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


public class Floor extends SubsystemBase 
{
    private static final int CONFIG_RETRIES = Constants.CONFIG_RETRIES;
    //Variables
    private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 60/18; //TODO TUNE
    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;
    private static final double MAX_FLOOR_SPEED_RPS = 100.0; // 100 is max rps for Kraken X60
    private double currentSpeedSetpointRps = 0.0;

    // PID
    private static final double SLOT0_KS = 0; // TODO TUNE
    private static final double SLOT0_KV = 0; // TODO TUNE
    private static final double SLOT0_KP = .1; // TODO TUNE
    private static final double SLOT0_KI = 0; // TODO TUNE
    private static final double SLOT0_KD = 0; // TODO TUNE
    
    // Spin Directions
    private static final InvertedValue LEFT_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    private static final InvertedValue RIGHT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    private static final MotorAlignmentValue RIGHT_MOTOR_ALIGNMENT = MotorAlignmentValue.Opposed;

    private final TalonFX left_motor;
    private final TalonFX  right_motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

public Floor() 
{
    left_motor= new TalonFX(Constants.CAN_ID.LEFT_FLOOR_ROLLER, Constants.CAN_BUS.CANIVORE);
    configureMotor(left_motor, LEFT_MOTOR_INVERTED, "Left Floor Motor");

    right_motor = new TalonFX(Constants.CAN_ID.RIGHT_FLOOR_ROLLER, Constants.CAN_BUS.CANIVORE);
    configureMotor(right_motor, RIGHT_MOTOR_INVERTED, "Right Floor Motor");
    right_motor.setControl(new Follower(Constants.CAN_ID.LEFT_FLOOR_ROLLER, RIGHT_MOTOR_ALIGNMENT));
}
    public void setSpeed(double speedRps) 
    {
        currentSpeedSetpointRps = speedRps;
        left_motor.setControl(velocityRequest.withVelocity(currentSpeedSetpointRps));
        // SmartDashboard.putNumber("Floor Setpoint RPS", currentSpeedSetpointRps);
    }

    public void setDumbSpeed(double percent)
    {
        left_motor.set(percent);
    }

    public void setFloorPercent(double percentOutput) 
    {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        setSpeed(clamped * MAX_FLOOR_SPEED_RPS);
    }

    public double getSpeed() 
    {
        return (left_motor.getVelocity().getValueAsDouble() + right_motor.getVelocity().getValueAsDouble()) / 2.0 ; 
    }

    public boolean isAtSpeed(double toleranceRps) 
    {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= Constants.Floor.SPEED_TOLERANCE_RPS;
    }

    public void stop() 
    {
        currentSpeedSetpointRps = 0.0;
        left_motor.stopMotor();
        // SmartDashboard.putNumber("Floor Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertedValue, String motorName) 
    {
        TalonFXConfiguration floorConfigs = new TalonFXConfiguration()
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

        floorConfigs.MotorOutput.Inverted = invertedValue;
        floorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        floorConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) 
        {
            status = motor.getConfigurator().apply(floorConfigs);
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
        // SmartDashboard.putNumber("Floor Current Speed RPS", getSpeed());
        // SmartDashboard.putNumber("Floor Speed Setpoint RPS", currentSpeedSetpointRps);
        // SmartDashboard.putBoolean("Floor Is At Speed", isAtSpeed(Constants.Floor.SPEED_TOLERANCE_RPS));
        SmartDashboard.putNumber(
                "Floor Current Draw Amps",
                (left_motor.getSupplyCurrent().getValueAsDouble()
                        + right_motor.getSupplyCurrent().getValueAsDouble()) / 2.0);
    }

}

