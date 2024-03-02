package frc2024.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private double m_targetVelocity = 0.0;

    public Shooter(){
        m_rightShooterMotor = new TalonFX(Ports.RIGHT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_leftShooterMotor = new TalonFX(Ports.LEFT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configShooterMotors();
        
        OrchestraUtil.add(m_rightShooterMotor, m_leftShooterMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Right Shooter Motor", m_rightShooterMotor, DeviceConfig.shooterFXConfig(InvertedValue.Clockwise_Positive), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Shooter Motor", m_leftShooterMotor, DeviceConfig.shooterFXConfig(InvertedValue.Clockwise_Positive), Constants.DEVICE_LOOP_TIME_HZ);
        m_rightShooterMotor.getRotorVelocity().setUpdateFrequency(50.0);
        m_leftShooterMotor.getRotorVelocity().setUpdateFrequency(50.0);
        ParentDevice.optimizeBusUtilizationForAll(m_rightShooterMotor, m_leftShooterMotor);
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
        m_targetVelocity = velocityRPM;
        setShooter(m_velocityRequest.withVelocity(velocityRPM / 60.0));
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

    public double getShooterError(){
        return m_targetVelocity - getRPM();
    }

    public double getTargetVelocity(){
        return m_targetVelocity;
    }

    public BooleanSupplier getShooterAtTarget(){
        return () -> Math.abs(getShooterError()) < ShooterConstants.TARGET_THRESHOLD;
    }

    public double getShooterCurrent(){
        return ScreamUtil.average(m_rightShooterMotor.getSupplyCurrent().getValueAsDouble(), m_leftShooterMotor.getSupplyCurrent().getValueAsDouble());
    }

    public double getShooterVoltage(){
        return ScreamUtil.average(m_rightShooterMotor.getSupplyVoltage().getValueAsDouble(), m_leftShooterMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void stop(){
        m_rightShooterMotor.stopMotor();
        m_leftShooterMotor.stopMotor();
    }

    @Override
    public void periodic() {
        //System.out.println("Shooter: " + getShooterAtTarget().getAsBoolean());
        //logOutputs();
    }

    public void logOutputs(){
        Logger.recordOutput("Shooter/Measured/AverageVelocity", getRPM());
        Logger.recordOutput("Shooter/Measured/AverageError", getShooterError());
        Logger.recordOutput("Shooter/Setpoint/Velocity", m_targetVelocity);
        Logger.recordOutput("Shooter/Setpoint/AtTarget", getShooterAtTarget().getAsBoolean());
        Logger.recordOutput("Shooter/Power/Left/SupplyCurrent", m_leftShooterMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/Power/Left/StatorCurrent", m_leftShooterMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/Power/Left/Voltage", m_leftShooterMotor.getSupplyVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/Power/Right/SupplyCurrent", m_rightShooterMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/Power/Right/StatorCurrent", m_rightShooterMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/Power/Right/Voltage", m_leftShooterMotor.getSupplyVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/Power/AverageVoltage", getShooterVoltage());
        Logger.recordOutput("Shooter/Power/AverageCurrent", getShooterCurrent());
        //Logger.recordOutput("Shooter/CurrentCommand", getCurrentCommand().getName());
    }

    public Command dutyCycleCommand(double dutyCycle){
        return run(() -> setShooterOutput(dutyCycle)).withName("DutyCycleCommand");
    }

    public Command velocityCommand(double velocityRPM){
        return run(() -> setTargetVelocity(velocityRPM)).withName("VelocityCommand");
    }

    public Command velocityCommand(DoubleSupplier velocityRPM){
        return run(() -> setTargetVelocity(velocityRPM.getAsDouble())).withName("VelocityCommand[Supplier]");
    }

    public Command idleCommand(){
        return velocityCommand(ShooterConstants.IDLE_VELOCITY).withName("IdleCommand");
    }

    public Command stopCommand(){
        return Commands.runOnce(() -> stop(), this).withName("StopCommand");
    }
}
