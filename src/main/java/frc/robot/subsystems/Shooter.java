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

public class Shooter extends SubsystemBase {

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
        private static final InvertedValue MOTOR_CLOCKWISE = InvertedValue.Clockwise_Positive;


    private static final double SPEED_TOLERANCE_RPS = 0.5;
    //private static final double MAX_ELEVATOR_SPEED_RPS = Constants.Shooter.MAX_ELEVATOR_SPEED_RPS;
        private final TalonFX left_shooter;
        private final TalonFX right_shooter;
        private final TalonFX left_middle_shooter;
        private final TalonFX right_middle_shooter;
    public Shooter() {
        left_shooter = new TalonFX(Constants.CAN_ID.BALL_ELEVATOR, Constants.CAN_BUS.CANIVORE);
        right_shooter = new TalonFX(Constants.CAN_ID.BALL_ELEVATOR, Constants.CAN_BUS.CANIVORE);
        left_middle_shooter = new TalonFX(Constants.CAN_ID.BALL_ELEVATOR, Constants.CAN_BUS.CANIVORE);
        right_middle_shooter = new TalonFX(Constants.CAN_ID.BALL_ELEVATOR, Constants.CAN_BUS.CANIVORE);
        configureMotor(left_shooter, MOTOR_INVERTED, "Left Shooter");
        configureMotor(right_shooter, MOTOR_CLOCKWISE, "Right Shooter");
        configureMotor(left_middle_shooter, MOTOR_INVERTED, "Left Middle Shooter");
        configureMotor(right_middle_shooter, MOTOR_CLOCKWISE, "Right Middle Shooter");

        // Initialize feeder-specific components here
    }

    @Override
    public void periodic() {
        super.periodic();
        // Add any feeder-specific periodic actions here
    }

    public void shoot(double speedRPS) {
        /* un edited code from previous robot TODO - edit to work with the 4 new shooter motors
        currentSpeedSetpointRps = Math.max(0.0, speedRps);
        shooterLeftMotor.setControl(velocityRequest.withVelocity(currentSpeedSetpointRps));
        shooterRightMotor.setControl(new Follower(Constants.CAN_ID.SHOOTER_LEFT, RIGHT_FOLLOW_ALIGNMENT));
        SmartDashboard.putNumber("Shooter Speed Setpoint RPS", currentSpeedSetpointRps);
        */
    }

    public void stopFeeding() {
        // Code to stop the feeder mechanism
        // This could involve stopping motors or other hardware specific to the feeder
    }
    
        private void configureMotor(TalonFX motor, InvertedValue invertedValue, String motorName) {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration()
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

        shooterConfigs.MotorOutput.Inverted = invertedValue;
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = motor.getConfigurator().apply(shooterConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for " + motorName + ", error code: " + status);
        }
    }
    
}
