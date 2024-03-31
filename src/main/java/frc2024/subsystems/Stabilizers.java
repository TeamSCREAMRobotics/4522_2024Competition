package frc2024.subsystems;

import java.text.Normalizer.Form;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.StabilizerConstants;
import frc2024.Constants.Ports;
import frc2024.Constants.RobotMode;

public class Stabilizers extends SubsystemBase{

  private TalonFX m_stabilizerMotor;

  private PositionVoltage m_positionRequest = new PositionVoltage(0);
  private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

  private double m_targetRotation;

  public Stabilizers (){
        m_stabilizerMotor = new TalonFX(Ports.BAR_MOTOR_ID);

        configureDevices();
        
        OrchestraUtil.add(m_stabilizerMotor);
    }
    
    private void configureDevices() {
        DeviceConfig.configureTalonFX("Bar Motor", m_stabilizerMotor, DeviceConfig.stabilizerFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_stabilizerMotor);
    }

    public void zeroRotation(){
        m_stabilizerMotor.setPosition(0.0);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_stabilizerMotor.setNeutralMode(mode);
    }

    public void setBar(ControlRequest control){
        m_stabilizerMotor.setControl(control);
    }

    public void setTargetPosition(double rotation){
        m_targetRotation = rotation;
        setBar(m_positionRequest.withPosition(m_targetRotation));
    }

    public void setBarOutput(double output){
        setBar(m_dutyCycleRequest.withOutput(output));
    }

    public void stopBar(){
        m_stabilizerMotor.stopMotor();
    }

    public double getBarRotation(){
        return m_stabilizerMotor.getPosition().refresh().getValue();
    }

    public double getBarError(){
        return m_targetRotation - getBarRotation();
    }

    public void stop(){
        m_stabilizerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if(Constants.MODE == RobotMode.COMP){
            logOutputs();
        }
    }

    public void logOutputs(){
        ScreamUtil.logBasicMotorOutputs("Stabilizers", m_stabilizerMotor);
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

    public Command stopCommand(){
        return runOnce(() -> stop());
    }
}