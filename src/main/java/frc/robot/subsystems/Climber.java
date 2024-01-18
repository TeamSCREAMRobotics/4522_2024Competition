package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.lib.util.OrchestraUtil;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.Ports;

public class Climber extends SubsystemBase{

  private TalonFX m_rightClimberMotor;
  private TalonFX m_leftClimberMotor;

  private double m_targetHeight;

  public Climber (){
        // m_leftClimberMotor = new TalonFX(Ports.RIGHT_CLIMBER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        // m_rightClimberMotor = new TalonFX(Ports.LEFT_CLIMBER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configClimberMotors();
        
        // OrchestraUtil.add(m_rightClimberMotor, m_leftClimberMotor);
    }
    
    private void configClimberMotors() {
        // DeviceConfig.configureTalonFX("Right Climber Motor", m_rightClimberMotor, DeviceConfig.elevatorFXConfig(), Constants.LOOP_TIME_HZ);
        // DeviceConfig.configureTalonFX("Left Climber Motor", m_leftClimberMotor, DeviceConfig.elevatorFXConfig(), Constants.LOOP_TIME_HZ);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_rightClimberMotor.setNeutralMode(mode);
        m_leftClimberMotor.setNeutralMode(mode);
    }

    public void zeroHeight(){
        m_rightClimberMotor.setPosition(0.0);
    }

    public void setTargetHeight(double height){
        m_targetHeight = height;
        setClimber(new MotionMagicVoltage(m_targetHeight));
    }
    
    public void setClimber(ControlRequest control){
        m_rightClimberMotor.setControl(control);
        m_leftClimberMotor.setControl(new Follower(m_rightClimberMotor.getDeviceID(), true)); //left motor follows right motor in the opposing direction
    }

    public void setClimberVoltage(double voltage){
        m_rightClimberMotor.setControl(new VoltageOut(voltage));
    }

    public void stopClimber(){
        m_rightClimberMotor.stopMotor();
    }

    public double getClimberHeight(){
        return m_rightClimberMotor.getPosition().refresh().getValue();
    }

    public double getClimberError(){
        return m_targetHeight - getClimberHeight();
    }
    
    public boolean getClimberAtTarget(){
        return Math.abs(getClimberError()) < ClimberConstants.TARGET_THRESHOLD;
    }

    public double getClimberVelocity(){
        return m_rightClimberMotor.getVelocity().getValue();
    }

    public double getClimberAcceleration(){
        return m_rightClimberMotor.getAcceleration().getValue();
    }

    public void stop(){
        m_rightClimberMotor.stopMotor();
        m_leftClimberMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }
}