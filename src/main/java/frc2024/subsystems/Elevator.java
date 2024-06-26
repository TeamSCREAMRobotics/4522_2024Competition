package frc2024.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.Ports;

public class Elevator extends SubsystemBase{
    
    private TalonFX m_rightElevatorMotor;
    private TalonFX m_leftElevatorMotor;

    private VoltageOut m_voltageRequest = new VoltageOut(0);
    private MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private double m_targetHeight;
    private double m_targetPosition;
    
    public Elevator(){
        m_rightElevatorMotor = new TalonFX(Ports.RIGHT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_leftElevatorMotor = new TalonFX(Ports.LEFT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configureDevices();

        OrchestraUtil.add(m_rightElevatorMotor, m_leftElevatorMotor);
    }
    
    private void configureDevices() {
        DeviceConfig.configureTalonFX("Right Elevator Motor", m_rightElevatorMotor, DeviceConfig.elevatorFXConfig(InvertedValue.Clockwise_Positive/* CounterClockwise_Positive */), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Elevator Motor", m_leftElevatorMotor, DeviceConfig.elevatorFXConfig(InvertedValue.CounterClockwise_Positive/* Clockwise_Positive */), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_leftElevatorMotor, m_rightElevatorMotor);
        zeroPosition();
    }

    public void configPID(ScreamPIDConstants constants){
        m_rightElevatorMotor.getConfigurator().apply(constants.toSlot0Configs(ElevatorConstants.FEEDFORWARD_CONSTANTS));
        m_leftElevatorMotor.getConfigurator().apply(constants.toSlot0Configs(ElevatorConstants.FEEDFORWARD_CONSTANTS));
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_rightElevatorMotor.setNeutralMode(mode);
        m_leftElevatorMotor.setNeutralMode(mode);
    }

    public void zeroPosition(){
        m_rightElevatorMotor.setPosition(0.0);
        m_leftElevatorMotor.setPosition(0.0);
    }
    
    public void setElevator(ControlRequest control){
        m_rightElevatorMotor.setControl(control);
        m_leftElevatorMotor.setControl(control);
    }

    public void setTargetHeight(double heightInches){
        m_targetHeight = heightInches;
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

    public double getElevatorCurrent(){
        return ScreamUtil.average(m_rightElevatorMotor.getSupplyCurrent().getValueAsDouble(), m_leftElevatorMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void stop(){
        m_rightElevatorMotor.stopMotor();
        m_leftElevatorMotor.stopMotor();
    }

    @Override
    public void periodic() {}

    public void logOutputs() {
        ScreamUtil.logBasicMotorOutputs("Elevator", m_leftElevatorMotor);
        ScreamUtil.logServoMotorOutputs("Elevator", m_leftElevatorMotor);
        Logger.recordOutput("Elevator/Measured/Height", getElevatorHeight());
        Logger.recordOutput("Elevator/Setpoint/Height", m_targetHeight);
        Logger.recordOutput("Elevator/Setpoint/Position", m_targetPosition);
        if(getCurrentCommand() != null){
            Logger.recordOutput("Elevator/CurrentCommand", getCurrentCommand().getName());
        }
    }

    public double heightInchesToPosition(double inches){
        return ((inches - ElevatorConstants.MIN_HEIGHT) / (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT)) * (ElevatorConstants.ENCODER_MAX - ElevatorConstants.ENCODER_MIN);
    }

    public Command voltageCommand(DoubleSupplier voltage){
        return run(() -> setElevatorVoltage(voltage.getAsDouble()))
            .alongWith(RobotContainer.getLED().mappedCommand(() -> Conversions.mapRange(getElevatorHeight(), 0, ElevatorConstants.MAX_HEIGHT, 0, 100))).withName("VoltageCommand");
    }

    public Command voltageCommand(double voltage){
        return voltageCommand(() -> voltage);
    }

    public Command heightCommand(double heightInches){
        return heightCommand(() -> heightInches);
    }

    public Command heightCommand(DoubleSupplier heightInches){
        return run(() -> setTargetHeight(heightInches.getAsDouble())).withName("HeightCommand");
    }

    public Command stopCommand(){
        return runOnce(() -> stop()).withName("StopCommand");    
    }
}
