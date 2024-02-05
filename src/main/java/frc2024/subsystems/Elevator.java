package frc2024.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2024.Constants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;

public class Elevator extends SubsystemBase{
    
    private TalonFX m_rightElevatorMotor;
    private TalonFX m_leftElevatorMotor;

    private double m_targetHeight;

    public Elevator(){
        // m_rightElevatorMotor = new TalonFX(Ports.RIGHT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        // m_leftElevatorMotor = new TalonFX(Ports.LEFT_ELEVATOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        // configElevatorMotors();

        m_leftElevatorMotor.setControl(new Follower(m_rightElevatorMotor.getDeviceID(), true)); //left motor follows right motor in the opposing direction
        //OrchestraUtil.add(m_rightElevatorMotor, m_leftElevatorMotor);
    }
    
    private void configElevatorMotors() {
        DeviceConfig.configureTalonFX("Right Elevator Motor", m_rightElevatorMotor, DeviceConfig.elevatorFXConfig(), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Left Elevator Motor", m_leftElevatorMotor, DeviceConfig.elevatorFXConfig(), Constants.LOOP_TIME_HZ);
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_rightElevatorMotor.setNeutralMode(mode);
        m_leftElevatorMotor.setNeutralMode(mode);
    }

    public void zeroHeight(){
        m_rightElevatorMotor.setPosition(0.0);
    }

    public void setTargetHeight(double height){
        m_targetHeight = height;
        setElevator(new MotionMagicVoltage(m_targetHeight));
    }
    
    public void setElevator(ControlRequest control){
        m_rightElevatorMotor.setControl(control);
    }

    public void setElevatorOutput(double po){
        setElevator(new DutyCycleOut(po));
    }

    public void setElevatorVoltage(double voltage){
        m_rightElevatorMotor.setControl(new VoltageOut(voltage));
    }
    
    public double getElevatorHeight(){
        return m_rightElevatorMotor.getPosition().refresh().getValue();
    }

    public double getElevatorError(){
        return m_targetHeight - getElevatorHeight();
    }
    
    public boolean getElevatorAtTarget(){
        return Math.abs(getElevatorError()) < ElevatorConstants.TARGET_THRESHOLD;
    }

    public double getElevatorVelocity(){
        return m_rightElevatorMotor.getVelocity().getValue();
    }

    public double getElevatorAcceleration(){
        return m_rightElevatorMotor.getAcceleration().getValue();
    }
    
    public double getElevatorTargetHeight(double distance){
        return ElevatorConstants.elevatorHeightMap_Localization.get(distance);
    }

    public void stop(){
        m_rightElevatorMotor.stopMotor();
        m_leftElevatorMotor.stopMotor();
    }
}