package frc2024.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;

public class Pivot extends SubsystemBase{
    
    private TalonFX m_pivotMotor;
    private CANcoder m_encoder;

    private MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);
    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

    public Pivot(){
        m_pivotMotor = new TalonFX(Ports.PIVOT_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_encoder = new CANcoder(Ports.PIVOT_ENCODER_ID, Ports.RIO_CANBUS_NAME);
         
        configureDevices();
        
        OrchestraUtil.add(m_pivotMotor);
    }

    private void configureDevices() {
        DeviceConfig.configureCANcoder("Pivot Encoder", m_encoder, DeviceConfig.pivotEncoderConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Pivot Motor", m_pivotMotor, DeviceConfig.pivotFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
    }

    public void configPID(ScreamPIDConstants constants){
        m_pivotMotor.getConfigurator().apply(constants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
    }

    public void zeroPivot(){
        m_pivotMotor.setPosition(0.0);
        m_encoder.setPosition(0.0);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_pivotMotor.setNeutralMode(mode);
    }

    public void setPivot(ControlRequest control){
        m_pivotMotor.setControl(control);
    }

    public void setTargetAngle(Rotation2d angle){
        m_targetAngle = angle;
        setPivot(m_positionRequest.withPosition(m_targetAngle.getRotations()));
    }

    public void setPivotOutput(double output){
        setPivot(m_dutyCycleRequest.withOutput(output));
    }

    public Rotation2d getPivotAngle(){
        return Rotation2d.fromRotations(m_pivotMotor.getPosition().refresh().getValue());
    }

    public Rotation2d getPivotError(){
        return m_targetAngle.minus(getPivotAngle());
    }

    public boolean getPivotAtTarget(){
        return Math.abs(getPivotError().getDegrees()) < PivotConstants.TARGET_THRESHOLD;
    }

    public double getPivotCurrent(){
        return m_pivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    public void stop(){
        m_pivotMotor.stopMotor();
    }

    @Override
    public void periodic() {}

    public Command dutyCycleCommand(DoubleSupplier dutyCycle){
        return run(() -> setPivotOutput(dutyCycle.getAsDouble()));
    }

    public Command dutyCycleCommand(double dutyCycle){
        return run(() -> setPivotOutput(dutyCycle));
    }

    public Command angleCommand(Rotation2d angle){
        return run(() -> setTargetAngle(angle));
    }

    public Command angleCommand(Supplier<Rotation2d> angle){
        return run(() -> setTargetAngle(angle.get()));
    }
}
