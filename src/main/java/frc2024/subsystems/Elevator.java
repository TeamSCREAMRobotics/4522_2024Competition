package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2024.Constants;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;

public class Elevator extends SubsystemBase{
    
    private TalonFX m_rightElevatorMotor;
    private TalonFX m_leftElevatorMotor;
    private CANcoder m_encoder;

    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private double m_targetHeight;

    public Elevator(){
        m_rightElevatorMotor = new TalonFX(Ports.RIGHT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_leftElevatorMotor = new TalonFX(Ports.LEFT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_encoder = new CANcoder(Ports.ELEVATOR_ENCODER_ID, Ports.RIO_CANBUS_NAME);

        configureDevices();

        //OrchestraUtil.add(m_rightElevatorMotor, m_leftElevatorMotor);
    }
    
    private void configureDevices() {
        DeviceConfig.configureCANcoder("Elevator Encoder", m_encoder, DeviceConfig.elevatorEncoderConfig(), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Right Elevator Motor", m_rightElevatorMotor, DeviceConfig.elevatorFXConfig(InvertedValue.Clockwise_Positive), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Elevator Motor", m_leftElevatorMotor, DeviceConfig.elevatorFXConfig(InvertedValue.CounterClockwise_Positive), Constants.LOOP_TIME_HZ);
        resetElevatorToAbsolute();
    }

    public void configPID(ScreamPIDConstants screamPIDConstants){
        m_rightElevatorMotor.getConfigurator().apply(screamPIDConstants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
        m_leftElevatorMotor.getConfigurator().apply(screamPIDConstants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
    }

    public void resetElevatorToAbsolute(){
        m_rightElevatorMotor.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble());
        m_leftElevatorMotor.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_rightElevatorMotor.setNeutralMode(mode);
        m_leftElevatorMotor.setNeutralMode(mode);
    }

    public void zeroHeight(){
        m_rightElevatorMotor.setPosition(0.0);
        m_leftElevatorMotor.setPosition(0.0);
    }

    public void setTargetPosition(double height){
        m_targetHeight = height;
        setElevator(m_positionRequest.withPosition(m_targetHeight));
    }
    
    public void setElevator(ControlRequest control){
        m_rightElevatorMotor.setControl(control);
        m_leftElevatorMotor.setControl(new Follower(m_rightElevatorMotor.getDeviceID(), true)); 
        //left motor follows right motor in the opposing direction
    }

    public void setElevatorVelocity(double velocity){
        setElevator(m_velocityRequest.withVelocity(velocity));
    }
    
    public double getElevatorPosition(){
        return ScreamUtil.average(m_rightElevatorMotor.getPosition().getValueAsDouble(), -m_leftElevatorMotor.getPosition().getValueAsDouble());
    }

    public double getElevatorError(){
        return m_targetHeight - getElevatorPosition();
    }
    
    public boolean getElevatorAtTarget(){
        return Math.abs(getElevatorError()) < ElevatorConstants.TARGET_THRESHOLD;
    }
    
    public double getElevatorTargetHeight(double distance){
        return ElevatorConstants.HEIGHT_MAP.get(distance);
    }

    public void stop(){
        m_rightElevatorMotor.stopMotor();
        m_leftElevatorMotor.stopMotor();
    }

    public Command outputCommand(DoubleSupplier output){
        return run(() -> setElevatorVelocity(output.getAsDouble()));
    }

    public Command outputCommand(double output){
        return run(() -> setElevatorVelocity(output));
    }

    public Command positionCommand(double position){
        return run(() -> setTargetPosition(position));
    }
}
