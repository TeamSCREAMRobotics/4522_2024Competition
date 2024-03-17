package frc2024.subsystems;

import java.util.function.BooleanSupplier;
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
import com.ctre.phoenix6.hardware.ParentDevice;
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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;

public class Elevator extends SubsystemBase{
    
    private TalonFX m_rightElevatorMotor;
    private TalonFX m_leftElevatorMotor;
    // private CANcoder m_encoder;

    private VoltageOut m_voltageRequest = new VoltageOut(0);
    private MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private double m_targetPosition;

    public Elevator(){
        m_rightElevatorMotor = new TalonFX(Ports.RIGHT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_leftElevatorMotor = new TalonFX(Ports.LEFT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        // m_encoder = new CANcoder(Ports.ELEVATOR_ENCODER_ID, Ports.RIO_CANBUS_NAME);

        configureDevices();

        OrchestraUtil.add(m_rightElevatorMotor, m_leftElevatorMotor);
    }
    
    private void configureDevices() {
        // DeviceConfig.configureCANcoder("Elevator Encoder", m_encoder, DeviceConfig.elevatorEncoderConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Right Elevator Motor", m_rightElevatorMotor, DeviceConfig.elevatorFXConfig(InvertedValue.Clockwise_Positive/* CounterClockwise_Positive */), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Elevator Motor", m_leftElevatorMotor, DeviceConfig.elevatorFXConfig(InvertedValue.CounterClockwise_Positive/* Clockwise_Positive */), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_leftElevatorMotor, m_rightElevatorMotor);
        zeroPosition();
    }

    public void configPID(ScreamPIDConstants constants){
        m_rightElevatorMotor.getConfigurator().apply(constants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
        m_leftElevatorMotor.getConfigurator().apply(constants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_rightElevatorMotor.setNeutralMode(mode);
        m_leftElevatorMotor.setNeutralMode(mode);
    }

    public void zeroPosition(){
        m_rightElevatorMotor.setPosition(0.0);
        m_leftElevatorMotor.setPosition(0.0);
        // m_encoder.setPosition(0.0);
    }
    
    public void setElevator(ControlRequest control){
        m_rightElevatorMotor.setControl(control);
        m_leftElevatorMotor.setControl(control);
    }

    public void setTargetHeight(double heightInches){
        setTargetPosition(heightInchesToPosition(heightInches));
    }

    public void setElevatorVoltage(double voltage){
        double volts = getElevatorHeight() < 0.25 ? voltage : voltage + ElevatorConstants.KG;
        setElevator(m_voltageRequest.withOutput(volts));
    }

    private void setTargetPosition(double positionRotations){
        m_targetPosition = positionRotations;
        setElevator(m_positionRequest.withPosition(positionRotations));
    }
    
    public double getElevatorPosition(){
        return ScreamUtil.average(m_rightElevatorMotor.getPosition().getValueAsDouble(), m_leftElevatorMotor.getPosition().getValueAsDouble());
    }

    public double getElevatorHeight(){
        return getElevatorPosition() * (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT) / (ElevatorConstants.ENCODER_MAX - ElevatorConstants.ENCODER_MIN);
    }

    public double getElevatorError(){
        return m_targetPosition - getElevatorPosition();
    }
    
    public BooleanSupplier getElevatorAtTarget(){
        return () -> Math.abs(getElevatorError()) < ElevatorConstants.TARGET_THRESHOLD;
    }
    
/*     public double getElevatorTargetHeight(double distance){
        return ElevatorConstants.HEIGHT_MAP.get(distance);
    } */

    public double getElevatorCurrent(){
        return ScreamUtil.average(m_rightElevatorMotor.getSupplyCurrent().getValueAsDouble(), m_leftElevatorMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void stop(){
        m_rightElevatorMotor.stopMotor();
        m_leftElevatorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        System.out.println(m_targetPosition);
    }

    public double heightInchesToPosition(double inches){
        return ((inches - ElevatorConstants.MIN_HEIGHT) / (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT)) * (ElevatorConstants.ENCODER_MAX - ElevatorConstants.ENCODER_MIN);
    }

    public Command voltageCommand(DoubleSupplier voltage){
        return run(() -> setElevatorVoltage(voltage.getAsDouble()));
            /* .alongWith(RobotContainer.getLED().scaledTargetCommand(Color.kYellow, () -> getElevatorHeight(), () -> ElevatorConstants.MAX_HEIGHT)); */
    }

    public Command voltageCommand(double voltage){
        return run(() -> setElevatorVoltage(voltage))
            .alongWith(RobotContainer.getLED().scaledTargetCommand(Color.kYellow, () -> getElevatorHeight(), () -> ElevatorConstants.MAX_HEIGHT));
    }

    public Command heightCommand(double heightInches){
        return run(() -> setTargetHeight(heightInches));
    }

    public Command heightCommand(DoubleSupplier heightInches){
        return run(() -> setTargetHeight(heightInches.getAsDouble()));
    }

    public Command reHomeCommand(){
        return voltageCommand(ElevatorConstants.REHOME_VOLTAGE)
            .until(() -> getElevatorCurrent() >= ElevatorConstants.REHOME_CURRENT_THRESHOLD)
            .finallyDo(() -> zeroPosition());
    }
}
