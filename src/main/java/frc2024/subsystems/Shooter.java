package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.Ports;
import frc2024.Constants.ShooterConstants;
import frc2024.dashboard.tabs.SubsystemTestTab;

public class Shooter extends SubsystemBase{
    
    private TalonFX m_rightShooterMotor;
    private TalonFX m_leftShooterMotor;

    private DutyCycleOut m_dutyCyleRequest = new DutyCycleOut(0);
    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public Shooter(){
        m_rightShooterMotor = new TalonFX(Ports.RIGHT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_leftShooterMotor = new TalonFX(Ports.LEFT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configShooterMotors();
        
        OrchestraUtil.add(m_rightShooterMotor, m_leftShooterMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Right Shooter Motor", m_rightShooterMotor, DeviceConfig.shooterFXConfig(InvertedValue.Clockwise_Positive), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Shooter Motor", m_leftShooterMotor, DeviceConfig.shooterFXConfig(InvertedValue.Clockwise_Positive), Constants.DEVICE_LOOP_TIME_HZ);
     }
    
    public void setNeutralMode(NeutralModeValue shooterMode){
        m_rightShooterMotor.setNeutralMode(shooterMode);
        m_leftShooterMotor.setNeutralMode(shooterMode);
    }

    public void setShooter(ControlRequest control){
        m_rightShooterMotor.setControl(control);
        m_leftShooterMotor.setControl(control);
    }

    public void setTargetVelocity(double velocityRPM){
        setShooter(m_velocityRequest.withVelocity(velocityRPM/60.0));
    }

    public void setShooterOutput(double dutyCyle){
        setShooter(m_dutyCyleRequest.withOutput(dutyCyle));
    }

    public double getFeetPerSecond(){
        return Conversions.rpmToFTPS(getRPM(), ShooterConstants.WHEEL_CIRCUMFERENCE);
    }

    public double getRPM(){
        return ScreamUtil.average(m_rightShooterMotor.getRotorVelocity().getValueAsDouble(), m_leftShooterMotor.getRotorVelocity().getValueAsDouble())*60;
    }

    public double getMotorVelocity(){
        return ScreamUtil.average(m_leftShooterMotor.getVelocity().getValueAsDouble(), m_rightShooterMotor.getVelocity().getValueAsDouble());
    }

    public void stop(){
        m_rightShooterMotor.stopMotor();
        m_leftShooterMotor.stopMotor();
    }

    @Override
    public void periodic() {}

    public Command dutyCycleCommand(double dutyCycle){
        return run(() -> setShooterOutput(dutyCycle));
    }

    public Command velocityCommand(double velocityRPM){
        return run(() -> setTargetVelocity(velocityRPM));
    }

    public Command velocityCommand(DoubleSupplier velocityRPM){
        return run(() -> setTargetVelocity(velocityRPM.getAsDouble()));
    }

    public Command stopCommand(){
        return run(() -> stop());
    }
}
