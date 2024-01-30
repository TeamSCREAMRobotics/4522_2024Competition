package frc.lib.config;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.ErrorChecker.DeviceConfiguration;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotionMagicConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SteerConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;

public class DeviceConfig {

    ////////////////////////////////////// CUSTOM CONFIGURATION METHODS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    public static TalonFXConfiguration driveFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.MotorOutput = FXMotorOutputConfig(DriveConstants.MOTOR_INVERT, DriveConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, DriveConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            DriveConstants.CURRENT_LIMIT_ENABLE, 
            DriveConstants.SUPPLY_CURRENT_LIMIT, 
            DriveConstants.SUPPLY_CURRENT_THRESHOLD, 
            DriveConstants.SUPPLY_TIME_THRESHOLD);
        config.Slot0 = FXPIDConfig(DriveConstants.PID_CONSTANTS);
        config.OpenLoopRamps = FXOpenLoopRampConfig(DriveConstants.OPEN_LOOP_RAMP);
        config.ClosedLoopRamps = FXClosedLoopRampConfig(DriveConstants.CLOSED_LOOP_RAMP);
        config.TorqueCurrent = FXTorqueCurrentConfig(DriveConstants.SLIP_CURRENT, -DriveConstants.SLIP_CURRENT, 0);
        return config;
    }

    public static TalonFXConfiguration steerFXConfig(int remoteSensorID){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.MotorOutput = FXMotorOutputConfig(SteerConstants.MOTOR_INVERT, SteerConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.FusedCANcoder, remoteSensorID, 1.0, SteerConstants.GEAR_RATIO, Rotation2d.fromRotations(0));
        config.ClosedLoopGeneral = FXClosedLoopGeneralConfig(true);
        config.CurrentLimits = FXCurrentLimitsConfig(
            SteerConstants.CURRENT_LIMIT_ENABLE, 
            SteerConstants.SUPPLY_CURRENT_LIMIT, 
            SteerConstants.SUPPLY_CURRENT_THRESHOLD, 
            SteerConstants.SUPPLY_TIME_THRESHOLD);
        config.MotionMagic = FXMotionMagicConfigs(SteerConstants.MOTION_MAGIC_CONSTANTS);
        config.Slot0 = FXPIDConfig(SteerConstants.PID_CONSTANTS);
        return config;
    }

    public static CANcoderConfiguration swerveEncoderConfig(Rotation2d angleOffset){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = SwerveConstants.MODULE_TYPE.CANcoderInvert;
        config.MagnetSensor.MagnetOffset = angleOffset.getRotations();
        return config;
    }

    public static Pigeon2Configuration swervePigeonConfig(){
        Pigeon2Configuration config = new Pigeon2Configuration();
        return config;
    }
    
    public static TalonFXConfiguration conveyorFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.MotorOutput = FXMotorOutputConfig(ConveyorConstants.MOTOR_INVERT, ConveyorConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, ConveyorConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            ConveyorConstants.CURRENT_LIMIT_ENABLE, 
            ConveyorConstants.SUPPLY_CURRENT_LIMIT, 
            ConveyorConstants.SUPPLY_CURRENT_THRESHOLD, 
            ConveyorConstants.SUPPLY_TIME_THRESHOLD);
        return config;
    }

    public static TalonFXConfiguration elevatorFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.SoftwareLimitSwitch = FXSoftwareLimitSwitchConfig(ElevatorConstants.SOFTWARE_LIMIT_ENABLE, ElevatorConstants.FORWARD_SOFT_LIMIT, ElevatorConstants.REVERSE_SOFT_LIMIT);
        config.MotorOutput = FXMotorOutputConfig(ElevatorConstants.MOTOR_INVERT, ElevatorConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, ElevatorConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            ElevatorConstants.CURRENT_LIMIT_ENABLE, 
            ElevatorConstants.SUPPLY_CURRENT_LIMIT, 
            ElevatorConstants.SUPPLY_CURRENT_THRESHOLD, 
            ElevatorConstants.SUPPLY_TIME_THRESHOLD);
        config.MotionMagic = FXMotionMagicConfigs(ElevatorConstants.MOTION_MAGIC_CONSTANTS);
        config.Slot0 = FXPIDConfig(ElevatorConstants.PID_CONSTANTS);
        return config;
    }

    public static TalonFXConfiguration intakeFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.MotorOutput = FXMotorOutputConfig(IntakeConstants.MOTOR_INVERT, IntakeConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, IntakeConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            IntakeConstants.CURRENT_LIMIT_ENABLE, 
            IntakeConstants.SUPPLY_CURRENT_LIMIT, 
            IntakeConstants.SUPPLY_CURRENT_THRESHOLD, 
            IntakeConstants.SUPPLY_TIME_THRESHOLD);
        return config;
    }

    public static TalonFXConfiguration pivotFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.SoftwareLimitSwitch = FXSoftwareLimitSwitchConfig(PivotConstants.SOFTWARE_LIMIT_ENABLE, PivotConstants.FORWARD_SOFT_LIMIT, PivotConstants.REVERSE_SOFT_LIMIT);
        config.MotorOutput = FXMotorOutputConfig(PivotConstants.MOTOR_INVERT, PivotConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, PivotConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            PivotConstants.CURRENT_LIMIT_ENABLE, 
            PivotConstants.SUPPLY_CURRENT_LIMIT, 
            PivotConstants.SUPPLY_CURRENT_THRESHOLD, 
            PivotConstants.SUPPLY_TIME_THRESHOLD);
        config.MotionMagic = FXMotionMagicConfigs(PivotConstants.MOTION_MAGIC_CONSTANTS);
        config.Slot0 = FXPIDConfig(PivotConstants.PID_CONSTANTS);
        return config;
    }
    
    public static TalonFXConfiguration shooterFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.MotorOutput = FXMotorOutputConfig(ShooterConstants.MOTOR_INVERT, ShooterConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, ShooterConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            ShooterConstants.CURRENT_LIMIT_ENABLE, 
            ShooterConstants.SUPPLY_CURRENT_LIMIT, 
            ShooterConstants.SUPPLY_CURRENT_THRESHOLD, 
            ShooterConstants.SUPPLY_TIME_THRESHOLD);
        config.Slot0 = FXPIDConfig(ShooterConstants.PID_CONSTANTS);
        return config;
    }

    public static TalonFXConfiguration climberFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio = FXAudioConfigs(false, false, true);
        config.SoftwareLimitSwitch = FXSoftwareLimitSwitchConfig(ClimberConstants.SOFTWARE_LIMIT_ENABLE, ClimberConstants.FORWARD_SOFT_LIMIT, ClimberConstants.REVERSE_SOFT_LIMIT);
        config.MotorOutput = FXMotorOutputConfig(ClimberConstants.MOTOR_INVERT, ClimberConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, ClimberConstants.GEAR_RATIO, 1.0, Rotation2d.fromRotations(0));
        config.CurrentLimits = FXCurrentLimitsConfig(
            ClimberConstants.CURRENT_LIMIT_ENABLE, 
            ClimberConstants.SUPPLY_CURRENT_LIMIT, 
            ClimberConstants.SUPPLY_CURRENT_THRESHOLD, 
            ClimberConstants.SUPPLY_TIME_THRESHOLD);
        config.MotionMagic = FXMotionMagicConfigs(ClimberConstants.MOTION_MAGIC_CONSTANTS);
        config.Slot0 = FXPIDConfig(ClimberConstants.PID_CONSTANTS);
        return config;
    }


    ////////////////////////////////////// GENERAL CONFIGURATION METHODS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    public static void configureTalonFX(String name, TalonFX motor, TalonFXConfiguration config, double updateFrequencyHz){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
                return ErrorChecker.hasConfiguredWithoutErrors(
                    motor.getConfigurator().apply(config),
                    motor.getConfigurator().setPosition(0),
                    motor.getDutyCycle().setUpdateFrequency(updateFrequencyHz),
                    motor.getPosition().setUpdateFrequency(updateFrequencyHz),
                    motor.getVelocity().setUpdateFrequency(updateFrequencyHz),
                    motor.optimizeBusUtilization());
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + motor.getDeviceID() + " version " + motor.getVersion(), true);
    }

    public static void configureSwerveEncoder(String name, CANcoder encoder, CANcoderConfiguration config, double updateFrequencyHz){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
                return ErrorChecker.hasConfiguredWithoutErrors(
                    encoder.getConfigurator().apply(config),
                    encoder.getPosition().setUpdateFrequency(updateFrequencyHz),
                    encoder.getAbsolutePosition().setUpdateFrequency(updateFrequencyHz),
                    encoder.optimizeBusUtilization()
                    );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + encoder.getDeviceID() + " version " + encoder.getVersion(), true);
    }

    public static void configurePigeon2(String name, Pigeon2 pigeon, Pigeon2Configuration config, double updateFrequencyHz){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
                return ErrorChecker.hasConfiguredWithoutErrors(
                    pigeon.getConfigurator().apply(config),
                    pigeon.setYaw(0),
                    pigeon.getYaw().setUpdateFrequency(updateFrequencyHz),
                    pigeon.getPitch().setUpdateFrequency(updateFrequencyHz),
                    pigeon.getRoll().setUpdateFrequency(updateFrequencyHz),
                    pigeon.getAngularVelocityZDevice().setUpdateFrequency(updateFrequencyHz),
                    pigeon.optimizeBusUtilization()
                    );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + pigeon.getDeviceID() + " version " + pigeon.getVersion(), true);
    }

    public static AudioConfigs FXAudioConfigs(boolean beepOnBoot, boolean beepOnConfig, boolean allowMusicDuringDisabled){
        AudioConfigs config = new AudioConfigs();
        config.BeepOnBoot = beepOnBoot;
        config.BeepOnConfig = beepOnConfig;
        config.AllowMusicDurDisable = allowMusicDuringDisabled;
        return config;
    }

    public static MotorOutputConfigs FXMotorOutputConfig(InvertedValue invert, NeutralModeValue neutralMode){
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.Inverted = invert;
        config.NeutralMode = neutralMode;
        return config;
    }

    public static FeedbackConfigs FXFeedbackConfig(FeedbackSensorSourceValue sensor, int remoteSensorID, double sensorToMechGR, double rotorToSensorGR, Rotation2d sensorOffset){
        FeedbackConfigs config = new FeedbackConfigs();
        config.FeedbackSensorSource = sensor;
        config.FeedbackRemoteSensorID = remoteSensorID;
        config.SensorToMechanismRatio = sensorToMechGR;
        config.RotorToSensorRatio = rotorToSensorGR;
        config.FeedbackRotorOffset = sensorOffset.getRotations();
        return config;
    }

    public static Slot0Configs FXPIDConfig(ScreamPIDConstants constants){
        return constants.toSlot0Configs();
    }

    public static OpenLoopRampsConfigs FXOpenLoopRampConfig(double ramp){
        OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
        config.DutyCycleOpenLoopRampPeriod = ramp;
        return config;
    }

    public static ClosedLoopRampsConfigs FXClosedLoopRampConfig(double ramp){
        ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();
        config.DutyCycleClosedLoopRampPeriod = ramp;
        return config;
    }

    public static CurrentLimitsConfigs FXCurrentLimitsConfig(boolean enable, double limit, double currentThreshold, double timeThreshold){
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = enable;
        config.SupplyCurrentLimit = limit;
        config.SupplyCurrentThreshold = currentThreshold;
        config.SupplyTimeThreshold = timeThreshold;
        return config;
    }

    public static ClosedLoopGeneralConfigs FXClosedLoopGeneralConfig(boolean continuousWrap){
        ClosedLoopGeneralConfigs config = new ClosedLoopGeneralConfigs();
        config.ContinuousWrap = continuousWrap;
        return config;
    }

    public static TorqueCurrentConfigs FXTorqueCurrentConfig(double peakForward, double peakReverse, double neutralDeadband){
        TorqueCurrentConfigs config = new TorqueCurrentConfigs();
        config.PeakForwardTorqueCurrent = peakForward;
        config.PeakReverseTorqueCurrent = peakReverse;
        config.TorqueNeutralDeadband = neutralDeadband;
        return config;
    }

    public static SoftwareLimitSwitchConfigs FXSoftwareLimitSwitchConfig(boolean eneable, double forwardLimit, double reverseLimit){
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = eneable;
        config.ReverseSoftLimitEnable = eneable;
        config.ForwardSoftLimitThreshold = forwardLimit;
        config.ReverseSoftLimitThreshold = reverseLimit;
        return config;
    }

    public static MotionMagicConfigs FXMotionMagicConfigs(MotionMagicConstants constants){
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = constants.acceleration();
        config.MotionMagicCruiseVelocity = constants.cruiseVelocity();
        config.MotionMagicJerk = constants.jerk();
        return config;
    }

    public static VoltageConfigs FXVoltageConfigs(double peakForwardVoltage, double peakReverseVoltage){
        VoltageConfigs config = new VoltageConfigs();
        config.PeakForwardVoltage = peakForwardVoltage;
        config.PeakReverseVoltage = peakReverseVoltage;
        return config;
    }
}
