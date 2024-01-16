package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.lib.math.Conversions;
import frc.lib.util.OrchestraUtil;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    
    private TalonFX m_rightShooterMotor;
    private TalonFX m_leftShooterMotor;

    public Shooter(){
        //m_rightShooterMotor = new TalonFX(Ports.RIGHT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        //m_leftShooterMotor = new TalonFX(Ports.LEFT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configShooterMotors();

        m_leftShooterMotor.setControl(new Follower(m_rightShooterMotor.getDeviceID(), true)); //left motor follows right motor in the opposing direction
        
        //OrchestraUtil.add(m_rightShooterMotor, m_leftShooterMotor);
    }
    
    private void configShooterMotors() {
        // DeviceConfig.configureTalonFX("rightShooterMotor", m_rightShooterMotor, DeviceConfig.shooterFXConfig(), Constants.LOOP_TIME_HZ);
        // DeviceConfig.configureTalonFX("leftShooterMotor", m_leftShooterMotor, DeviceConfig.shooterFXConfig(), Constants.LOOP_TIME_HZ);
    }
    
    public void setNeutralMode(NeutralModeValue shooterMode){
        m_rightShooterMotor.setNeutralMode(shooterMode);
        m_leftShooterMotor.setNeutralMode(shooterMode);
    }

    public void setTargetVelocity(double rpm){
        m_rightShooterMotor.setControl(new VelocityVoltage(Conversions.rpmToFalconRPS(rpm, 1.0)));
    }

    public void setShooterPercentOutput(double output){
        m_rightShooterMotor.setControl(new DutyCycleOut(output));
    }

    public void setShooter(ControlRequest control){
        m_rightShooterMotor.setControl(control);
    }

    public void stop(){
        m_rightShooterMotor.stopMotor();
        m_leftShooterMotor.stopMotor();
    }

    @Override
    public void periodic(){

    }
}
