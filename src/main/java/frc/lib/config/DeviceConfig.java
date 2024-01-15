package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
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
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SteerConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;

public class DeviceConfig {

    ////////////////////////////////////// CUSTOM CONFIGURATION METHODS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    public static TalonFXConfiguration driveFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.Audio.AllowMusicDurDisable = true;
        config.MotorOutput = FXMotorOutputConfig(DriveConstants.MOTOR_INVERT, DriveConstants.NEUTRAL_MODE);
        config.Feedback = FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, DriveConstants.GEAR_RATIO, Rotation2d.fromRotations(0));
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
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.Audio.AllowMusicDurDisable = true;
        config.MotorOutput = FXMotorOutputConfig(SteerConstants.MOTOR_INVERT, SteerConstants.NEUTRAL_MODE);
        config.Feedback = FXSteerFeedbackConfig(FeedbackSensorSourceValue.FusedCANcoder, remoteSensorID, SteerConstants.GEAR_RATIO, Rotation2d.fromRotations(0));
        config.ClosedLoopGeneral = FXClosedLoopGeneralConfig(true);
        config.CurrentLimits = FXCurrentLimitsConfig(
            SteerConstants.CURRENT_LIMIT_ENABLE, 
            SteerConstants.SUPPLY_CURRENT_LIMIT, 
            SteerConstants.SUPPLY_CURRENT_THRESHOLD, 
            SteerConstants.SUPPLY_TIME_THRESHOLD);
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
        config.MountPose.MountPoseYaw = -90;
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
                    motor.optimizeBusUtilization()
                    );
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

    public static MotorOutputConfigs FXMotorOutputConfig(InvertedValue invert, NeutralModeValue neutralMode){
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.Inverted = invert;
        config.NeutralMode = neutralMode;
        return config;
    }

    public static FeedbackConfigs FXFeedbackConfig(FeedbackSensorSourceValue sensor, int remoteSensorID, double sensorToMechGR, Rotation2d sensorOffset){
        FeedbackConfigs config = new FeedbackConfigs();
        config.FeedbackSensorSource = sensor;
        config.FeedbackRemoteSensorID = remoteSensorID;
        config.SensorToMechanismRatio = sensorToMechGR;
        config.FeedbackRotorOffset = sensorOffset.getRotations();
        return config;
    }

    public static FeedbackConfigs FXSteerFeedbackConfig(FeedbackSensorSourceValue sensor, int remoteSensorID, double rotorToSensorGR, Rotation2d sensorOffset){
        FeedbackConfigs config = new FeedbackConfigs();
        config.FeedbackSensorSource = sensor;
        config.FeedbackRemoteSensorID = remoteSensorID;
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
}
