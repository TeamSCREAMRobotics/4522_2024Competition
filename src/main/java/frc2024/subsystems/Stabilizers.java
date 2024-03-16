package frc2024.subsystems;

import java.text.Normalizer.Form;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.Ports;

public class Stabilizers extends SubsystemBase{

  //private TalonFX m_climberMotor;
  private TalonFX m_barMotor;

    // private TalonSRXControlMode m_dutyCycleRequest = TalonSRXControlMode.PercentOutput;
    // private TalonSRXControlMode m_positionRequest = TalonSRXControlMode.Position;

  private PositionVoltage m_positionRequest = new PositionVoltage(0);
  private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    // private double m_targetHeight;
  private double m_targetRotation;

  public Stabilizers (){
        //m_climberMotor = new TalonFX(Ports.LEFT_CLIMBER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_barMotor = new TalonFX(Ports.BAR_MOTOR_ID);

        configureDevices();
        
        //OrchestraUtil.add(m_climberMotor);
    }
    
    private void configureDevices() {
        // DeviceConfig.configureTalonSRX("Bar Motor", m_barMotor, DeviceConfig.barSRXConfig());
        DeviceConfig.configureTalonFX("Bar Motor", m_barMotor, DeviceConfig.climberFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        // DeviceConfig.configureTalonFX("Climber Motor", m_climberMotor, DeviceConfig.climberFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        //ParentDevice.optimizeBusUtilizationForAll(m_climberMotor);
    }

    /* public void setBar(TalonSRXControlMode control, double value){
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
    } */

    public void configPID(ScreamPIDConstants constants){
        m_barMotor.getConfigurator().apply(constants.toSlot0Configs(ClimberConstants.FEEDFORWARD_CONSTANTS));
    }

    public void zeroRotation(){
        m_barMotor.setPosition(0.0);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_barMotor.setNeutralMode(mode);
    }

    public void setBar(ControlRequest control){
        m_barMotor.setControl(control);
    }

    public void setTargetPosition(double rotation){
        m_targetRotation = rotation;
        setBar(m_positionRequest.withPosition(m_targetRotation));
    }

    public void setBarOutput(double output){
        setBar(m_dutyCycleRequest.withOutput(output));
    }

    public void stopBar(){
        m_barMotor.stopMotor();
    }

    public double getBarRotation(){
        return m_barMotor.getPosition().refresh().getValue();
    }

    public double getBarError(){
        return m_targetRotation - getBarRotation();
    }

    /* public boolean getClimberAtBottom(){
        return m_barMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getClimberAtTop(){
        return m_barMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    } */
    
    public boolean getBarAtTarget(){
        return Math.abs(getBarError()) < ClimberConstants.TARGET_THRESHOLD;
    }

    @Override
    public void periodic() {}

    public void stop(){
        m_barMotor.stopMotor();
    }

    public Command outputCommand(DoubleSupplier output){
        return run(() -> setBarOutput(output.getAsDouble()));
    }

    public Command outputCommand(double output){
        return run(() -> setBarOutput(output));
    }

    public Command positionCommand(double position){
        return run(() -> setTargetPosition(position));
    }
}