package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    
    private TalonFX m_rightShooterMotor;;
    private TalonFX m_leftShooterMotor;

    public Shooter(){
        m_rightShooterMotor = new TalonFX(Ports.RIGHT_SHOOTER_MOTOR_ID, Ports.CANIVORE_NAME);
        m_leftShooterMotor = new TalonFX(Ports.LEFT_SHOOTER_MOTOR_ID, Ports.CANIVORE_NAME);

        configShooterMotors();

        m_leftShooterMotor.setControl(new Follower(m_rightShooterMotor.getDeviceID(), true)); //leftShooterMotor follows rightShooterMotor in the opposing direction
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("rightShooterMotor", m_rightShooterMotor, null, Constants.LOOP_TIME_HZ); //TODO create shooter config
        DeviceConfig.configureTalonFX("leftShooterMotor", m_leftShooterMotor, null, Constants.LOOP_TIME_HZ); //TODO create shooter config
    }
    
    public void setNeutralModes(NeutralModeValue shooterMode){
        m_rightShooterMotor.setNeutralMode(shooterMode);
        m_leftShooterMotor.setNeutralMode(shooterMode);
    }

    public void setShooterOutput(double output){
        m_rightShooterMotor.set(output);
    }

    public void stopMotor(){
        m_rightShooterMotor.stopMotor();
    }
    
    public double getShooterOutput(double distance){
        return ShooterConstants.shooterTreeMap.get(distance);
    }

    @Override
    public void periodic(){

    }
}
