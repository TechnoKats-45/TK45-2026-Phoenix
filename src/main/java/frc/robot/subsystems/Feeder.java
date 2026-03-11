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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 1;
    private static final double MM_CRUISE_RPS = 120;
    private static final double MM_ACCEL_RPS2 = 5;

    private static final double SLOT0_KS = 0; // TODO - tune
    private static final double SLOT0_KV = 0.0; // TODO - tune
    private static final double SLOT0_KP = .55; // TODO - tune
    private static final double SLOT0_KI = 0.0; // TODO - tune
    private static final double SLOT0_KD = 0.0; // TODO - tune

    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;
    private static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;


    private static final double SPEED_TOLERANCE_RPS = 0.5;
    //private static final double MAX_ELEVATOR_SPEED_RPS = Constants.Shooter.MAX_ELEVATOR_SPEED_RPS;
        private final TalonFX feeder1;
        private final TalonFX feeder2;
    public Feeder() {
        feeder1 = new TalonFX(Constants.CAN_ID.BALL_ELEVATOR, Constants.CAN_BUS.CANIVORE);
        feeder2 = new TalonFX(Constants.CAN_ID.BALL_ELEVATOR, Constants.CAN_BUS.CANIVORE);
        configureMotor();

        // Initialize feeder-specific components here
    }

    @Override
    public void periodic() {
        super.periodic();
        // Add any feeder-specific periodic actions here
    }

    public void feed() {
        // Code to activate the feeder mechanism
        // This could involve controlling motors or other hardware specific to the feeder
    }

    public void stopFeeding() {
        // Code to stop the feeder mechanism
        // This could involve stopping motors or other hardware specific to the feeder
    }
       private void configureMotor() {
        TalonFXConfiguration ballElevatorConfigs = new TalonFXConfiguration()
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
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));
        MotionMagicConfigs mm = ballElevatorConfigs.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2));
        // .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));

        ballElevatorConfigs.MotorOutput.Inverted = MOTOR_INVERTED;
        ballElevatorConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = ballElevatorMotor.getConfigurator().apply(ballElevatorConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for ball elevator, error code: " + status);
        }
    }
    
}
