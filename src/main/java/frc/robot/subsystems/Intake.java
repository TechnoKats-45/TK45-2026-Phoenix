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

public class Intake extends SubsystemBase 
{   
    private static final int CONFIG_RETRIES = Constants.CONFIG_RETRIES;
    
    // Pivot Vars

    private static final double STATOR_CURRENT_LIMIT_AMPS_PIVOT = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS_PIVOT = 60.0;
    private static final double MM_CRUISE_RPS_PIVOT = 1; // TODO TUNE
    private static final double MM_ACCEL_RPS2_PIVOT = 10; // TODO TUNE
    private static final double SENSOR_TO_MECHANISM_RATIO_PIVOT = 128.0; // TODO TUNE
    private static final double PEAK_FORWARD_VOLTS_PIVOT= 16.0;
    private static final double PEAK_REVERSE_VOLTS_PIVOT= - 16.0; 
    private double currentAngleSetPoint = 0.0;   

    // PIVOT PID
    private static final double SLOT0_KS = 0.0; // TODO TUNE
    private static final double SLOT0_KV = 0.0; // TODO TUNE
    private static final double SLOT0_KP = 1.0; // TODO TUNE
    private static final double SLOT0_KI = 0.0; // TODO TUNE
    private static final double SLOT0_KD = 1.0; // TODO TUNE
    
    //Pivot Inverted Values TODO TUNE
    private static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive; //TODO TUNE



    //Roller Vars
    private static final double STATOR_CURRENT_LIMIT_AMPS_ROLLER = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS_ROLLER = 60.0;
    private static final double MM_CRUISE_RPS_ROLLER = 50.0; // TODO TUNE
    private static final double MM_ACCEL_RPS2_ROLLER = 100.0; // TODO TUNE
    private static final double SENSOR_TO_MECHANISM_RATIO_ROLLER = 1.75; //TODO TUNE
    private static final double PEAK_FORWARD_VOLTS_ROLLER = 16.0;
    private static final double PEAK_REVERSE_VOLTS_ROLLER = -16.0;
    private static final double MAX_INTAKE_SPEED_RPS = 400.0; // TODO TUNE

    private double currentSpeedSetpointRps = 0.0;
    
    // ROLLERS PID
    private static final double SLOT1_KS = 0.0; //  TODO TUNE
    private static final double SLOT1_KV = 0.0; // TODO TUNE
    private static final double SLOT1_KP = 100.0; // TODO TUNE
    private static final double SLOT1_KI = 0.0; // TODO TUNE
    private static final double SLOT1_KD = 1.0; // TODO TUNE

    //Roller Inverted Values TODO TUNE
    private static final InvertedValue LEFT_ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;
    private static final InvertedValue RIGHT_ROLLER_INVERTED = InvertedValue.Clockwise_Positive;
    private static final MotorAlignmentValue RIGHT_FOLLOW_ROLLER_ALIGNMENT = MotorAlignmentValue.Opposed;

    // declare pivot motors
    private final TalonFX intake_pivot_motor;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    // declare rollor motor
    private final TalonFX intake_left_roller_motor;
    private final TalonFX intake_right_roller_motor;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    

//configure all motors and set right motors as followers
    public Intake()
    {
        intake_pivot_motor = new TalonFX(Constants.CAN_ID.INTAKE_PIVOT, Constants.CAN_BUS.CANIVORE);
        configurePivotMotor(intake_pivot_motor, PIVOT_INVERTED, "Pivot");
        
        
        intake_left_roller_motor = new TalonFX(Constants.CAN_ID.INTAKE_LEFT_ROLLER, Constants.CAN_BUS.RIO);
        configureRollerMotor(intake_left_roller_motor, LEFT_ROLLER_INVERTED, "Left Roller");
        
        intake_right_roller_motor = new TalonFX(Constants.CAN_ID.INTAKE_RIGHT_ROLLER, Constants.CAN_BUS.RIO);
        configureRollerMotor(intake_right_roller_motor, RIGHT_ROLLER_INVERTED, "Right Roller");
        intake_right_roller_motor.setControl(new Follower(Constants.CAN_ID.INTAKE_LEFT_ROLLER, RIGHT_FOLLOW_ROLLER_ALIGNMENT));
    }
    
    public void zeroEncoder() 
    {
        intake_pivot_motor.setPosition(0.0); 
        currentAngleSetPoint = 0.0; 
    }

    // pivot code

    public void setAngle(double angle)
    {
        currentAngleSetPoint = angle;
        intake_pivot_motor.setControl(motionMagicVoltage.withPosition(angle));
        SmartDashboard.putNumber("Intake Set Point", angle);
    }

    public double getAngle()
    {
        return intake_pivot_motor.getPosition().getValueAsDouble();
    }

    public boolean isAligned()
    {
        return Math.abs(getAngle() - currentAngleSetPoint) <= Constants.Intake.ANGLE_TOLERANCE_DEGREES;
    }
    
    public void setSpeed(double speedRps) 
    {
        currentSpeedSetpointRps = speedRps;
        intake_left_roller_motor.setControl(velocityRequest.withVelocity(currentSpeedSetpointRps));
        SmartDashboard.putNumber("Intake Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    
    public void runFeed(double percentOutput) 
    {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        setSpeed(clamped * MAX_INTAKE_SPEED_RPS);
    }

    public double getSpeed() 
    {
        return (intake_left_roller_motor.getVelocity().getValueAsDouble() + intake_right_roller_motor.getVelocity().getValueAsDouble()) / 2.0 ; 
    }


    public boolean isAtSpeed(double toleranceRps) 
    {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= Constants.Intake.SPEED_TOLERANCE_RPS;
    }

    public void stop() 
    {
        currentSpeedSetpointRps = 0.0;
        intake_left_roller_motor.stopMotor();
        SmartDashboard.putNumber("Intake Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    public void stow() 
    {
        setAngle(Constants.Intake.PIVOT_ANGLE_UP_STOWED);
    }

    public void setBrakeMode(boolean enableBrake)
    {
        NeutralModeValue mode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        intake_pivot_motor.setNeutralMode(mode);
        
    }
    public void setDumbSpeed(double percent) 
    {
        runFeed(percent);
    }

    public void setIntakePercent(double percentOutput) 
    {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        setSpeed(clamped * MAX_INTAKE_SPEED_RPS);
    }

    private void configurePivotMotor(TalonFX motor, InvertedValue invertedValue, String motorName) 
    {
        TalonFXConfiguration intakePivotConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS_PIVOT)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS_PIVOT)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_PIVOT))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS_PIVOT))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2_PIVOT)))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT0_KS)
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));

        intakePivotConfigs.MotorOutput.Inverted = invertedValue;
        intakePivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakePivotConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS_PIVOT))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS_PIVOT));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = motor.getConfigurator().apply(intakePivotConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for " + motorName + ", error code: " + status);
        }
    }

    private void configureRollerMotor(TalonFX motor, InvertedValue invertedValue, String motorName) {
        TalonFXConfiguration intakePivotConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS_ROLLER)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS_ROLLER)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_ROLLER))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS_ROLLER))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2_ROLLER)))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT1_KS)
                        .withKV(SLOT1_KV)
                        .withKP(SLOT1_KP)
                        .withKI(SLOT1_KI)
                        .withKD(SLOT1_KD));

        intakePivotConfigs.MotorOutput.Inverted = invertedValue;
        intakePivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakePivotConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS_ROLLER))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS_ROLLER));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = motor.getConfigurator().apply(intakePivotConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for " + motorName + ", error code: " + status);
        }
    }

    public void printDiagnostics() {
        SmartDashboard.putNumber("Intake Current Angle", getAngle());
        SmartDashboard.putNumber("Intake Angle Setpoint", currentAngleSetPoint);
        SmartDashboard.putBoolean("Intake Is Aligned", isAligned());
        SmartDashboard.putNumber(" Intake Current Speed RPS", getSpeed());
        SmartDashboard.putNumber(" Intake Speed Setpoint RPS", currentSpeedSetpointRps);
        SmartDashboard.putBoolean("Intake Is At Speed", isAtSpeed(Constants.Intake.SPEED_TOLERANCE_RPS));
        SmartDashboard.putNumber("Intake Pivot Current", intake_pivot_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller Current", intake_left_roller_motor.getSupplyCurrent().getValueAsDouble());
    }
}