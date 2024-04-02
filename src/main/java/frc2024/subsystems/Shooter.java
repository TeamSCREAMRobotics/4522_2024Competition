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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.Ports;
import frc2024.Constants.RobotMode;
import frc2024.Constants.ShooterConstants;
import frc2024.dashboard.tabs.SubsystemTestTab;
import frc2024.subsystems.Vision.Limelight;

public class Shooter extends SubsystemBase{
    
    private TalonFX m_bottomShooterMotor;
    private TalonFX m_topShooterMotor;

    private DutyCycleOut m_dutyCyleRequest = new DutyCycleOut(0);
    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    private double m_targetVelocity = 0.0;

    public Shooter(){
        m_bottomShooterMotor = new TalonFX(Ports.BOTTOM_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_topShooterMotor = new TalonFX(Ports.TOP_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configShooterMotors();
        
        OrchestraUtil.add(m_bottomShooterMotor, m_topShooterMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Bottom Shooter Motor", m_bottomShooterMotor, DeviceConfig.shooterFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Top Shooter Motor", m_topShooterMotor, DeviceConfig.shooterFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        m_bottomShooterMotor.getRotorVelocity().setUpdateFrequency(50.0);
        m_topShooterMotor.getRotorVelocity().setUpdateFrequency(50.0);
        ParentDevice.optimizeBusUtilizationForAll(m_bottomShooterMotor, m_topShooterMotor);
     }
    
    public void setNeutralMode(NeutralModeValue shooterMode){
        m_bottomShooterMotor.setNeutralMode(shooterMode);
        m_topShooterMotor.setNeutralMode(shooterMode);
    }

    public void setShooter(ControlRequest control){
        m_bottomShooterMotor.setControl(control);
        m_topShooterMotor.setControl(control);
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
        return ScreamUtil.average(m_bottomShooterMotor.getRotorVelocity().getValueAsDouble(), m_topShooterMotor.getRotorVelocity().getValueAsDouble())*60;
    }

    public double getMotorVelocity(){
        return ScreamUtil.average(m_topShooterMotor.getVelocity().getValueAsDouble(), m_bottomShooterMotor.getVelocity().getValueAsDouble());
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
        return ScreamUtil.average(m_bottomShooterMotor.getSupplyCurrent().getValueAsDouble(), m_topShooterMotor.getSupplyCurrent().getValueAsDouble());
    }

    public double getShooterVoltage(){
        return ScreamUtil.average(m_bottomShooterMotor.getSupplyVoltage().getValueAsDouble(), m_topShooterMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void stop(){
        m_bottomShooterMotor.stopMotor();
        m_topShooterMotor.stopMotor();
    }

    @Override
    public void periodic() {
        //System.out.println("Shooter Error: " + getShooterError());
        if(Constants.MODE == RobotMode.COMP){
            logOutputs();
        }
    }

    public void logOutputs(){
        Logger.recordOutput("Shooter/Measured/AverageVelocity", getRPM());
        Logger.recordOutput("Shooter/Measured/AverageError", getShooterError());
        Logger.recordOutput("Shooter/Setpoint/Velocity", m_targetVelocity);
        Logger.recordOutput("Shooter/Setpoint/AtTarget", getShooterAtTarget().getAsBoolean());
        ScreamUtil.logBasicMotorOutputs("Shooter/Bottom", m_bottomShooterMotor);
        ScreamUtil.logBasicMotorOutputs("Shooter/Top", m_topShooterMotor);
        //Logger.recordOutput("Shooter/CurrentCommand", getCurrentCommand().getName());
    }

    public Command dutyCycleCommand(double dutyCycle){
        return run(() -> setShooterOutput(dutyCycle)).withName("DutyCycleCommand");
    }

    public Command velocityCommand(double velocityRPM){
        return velocityCommand(() -> velocityRPM);
    }

    public Command velocityCommand(DoubleSupplier velocityRPM){
        return run(() -> setTargetVelocity(velocityRPM.getAsDouble())).withName("VelocityCommand")
            .alongWith(RobotContainer.getLED().scaledTargetCommand(Color.kPurple, () -> getRPM(), velocityRPM));
    }

    public Command idleCommand(){
        return run(() -> setTargetVelocity(ShooterConstants.IDLE_VELOCITY)).withName("IdleCommand");
    }

    public Command stopCommand(){
        return Commands.runOnce(() -> stop(), this).withName("StopCommand");
    }
}
