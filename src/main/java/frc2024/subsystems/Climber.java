package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.Ports;

public class Climber extends SubsystemBase{

  private TalonFX m_rightClimberMotor;
  private TalonFX m_leftClimberMotor;

  private double m_targetHeight;

  public Climber (){
        m_leftClimberMotor = new TalonFX(Ports.RIGHT_CLIMBER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_rightClimberMotor = new TalonFX(Ports.LEFT_CLIMBER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configClimberMotors();
        
        // OrchestraUtil.add(m_rightClimberMotor, m_leftClimberMotor);
    }
    
    private void configClimberMotors() {
        DeviceConfig.configureTalonFX("Right Climber Motor", m_rightClimberMotor, DeviceConfig.climberFXConfig(), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Climber Motor", m_leftClimberMotor, DeviceConfig.climberFXConfig(), Constants.LOOP_TIME_HZ);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_rightClimberMotor.setNeutralMode(mode);
        m_leftClimberMotor.setNeutralMode(mode);
    }

    public void zeroHeight(){
        m_rightClimberMotor.setPosition(0.0);
    }

    public void setTargetPosition(double height){
        m_targetHeight = height;
        setClimber(new MotionMagicVoltage(m_targetHeight));
    }
    
    public void setClimber(ControlRequest control){
        m_rightClimberMotor.setControl(control);
        m_leftClimberMotor.setControl(new Follower(m_rightClimberMotor.getDeviceID(), false)); //left motor follows right motor in the opposing direction
    }

    public void setClimberOutput(double po){
        setClimber(new DutyCycleOut(po));
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

    public void stop(){
        m_rightClimberMotor.stopMotor();
        m_leftClimberMotor.stopMotor();
    }

    public Command outputCommand(DoubleSupplier output){
        return run(() -> setClimberOutput(output.getAsDouble()));
    }

    public Command outputCommand(double output){
        return run(() -> setClimberOutput(output));
    }

    public Command positionCommand(double position){
        return run(() -> setTargetPosition(position));
    }
}