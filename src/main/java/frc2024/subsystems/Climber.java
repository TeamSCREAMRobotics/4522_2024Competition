package frc2024.subsystems;

import java.text.Normalizer.Form;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.Ports;

public class Climber extends SubsystemBase{

  //private TalonFX m_climberMotor;
  private TalonSRX m_barMotor;

  private TalonSRXControlMode m_dutyCycleRequest = TalonSRXControlMode.PercentOutput;
  private TalonSRXControlMode m_positionRequest = TalonSRXControlMode.Position;

  //private PositionVoltage m_positionRequest = new PositionVoltage(0);
  //private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

  //private double m_targetHeight;

  public Climber (){
        //m_climberMotor = new TalonFX(Ports.LEFT_CLIMBER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_barMotor = new TalonSRX(Ports.BAR_MOTOR_ID);

        configureDevices();
        
        //OrchestraUtil.add(m_climberMotor);
    }
    
    private void configureDevices() {
        DeviceConfig.configureTalonSRX("Bar Motor", m_barMotor, DeviceConfig.barSRXConfig());
        //DeviceConfig.configureTalonFX("Climber Motor", m_climberMotor, DeviceConfig.climberFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        //ParentDevice.optimizeBusUtilizationForAll(m_climberMotor);
    }

    public void setBar(TalonSRXControlMode control, double value){
        m_barMotor.set(control, value);
    }

    public double getBarPosition(){
        return m_barMotor.getSelectedSensorPosition();
    }

    public Command barPositionCommand(double position){
        return run(() -> setBar(m_positionRequest, position));
    }

    public Command barDutyCycleCommand(double dutyCycle){
        return run(() -> setBar(m_dutyCycleRequest, dutyCycle));
    }

    /* public void configPID(ScreamPIDConstants constants){
        m_climberMotor.getConfigurator().apply(constants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
    }

    public void zeroHeight(){
        m_climberMotor.setPosition(0.0);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_climberMotor.setNeutralMode(mode);
    }

    public void setClimber(ControlRequest control){
        m_climberMotor.setControl(control);
    }

    public void setTargetPosition(double height){
        m_targetHeight = height;
        setClimber(m_positionRequest.withPosition(m_targetHeight));
    }

    public void setClimberOutput(double output){
        setClimber(m_dutyCycleRequest.withOutput(output));
    }

    public void stopClimber(){
        m_climberMotor.stopMotor();
    }

    public double getClimberHeight(){
        return m_climberMotor.getPosition().refresh().getValue();
    }

    public double getClimberError(){
        return m_targetHeight - getClimberHeight();
    }

    public boolean getClimberAtBottom(){
        return m_climberMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getClimberAtTop(){
        return m_climberMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }
    
    public boolean getClimberAtTarget(){
        return Math.abs(getClimberError()) < ClimberConstants.TARGET_THRESHOLD;
    }

    @Override
    public void periodic() {}

    public void stop(){
        m_climberMotor.stopMotor();
    }

    public Command outputCommand(DoubleSupplier output){
        return run(() -> setClimberOutput(output.getAsDouble()));
    }

    public Command outputCommand(double output){
        return run(() -> setClimberOutput(output));
    }

    public Command positionCommand(double position){
        return run(() -> setTargetPosition(position));
    } */
}