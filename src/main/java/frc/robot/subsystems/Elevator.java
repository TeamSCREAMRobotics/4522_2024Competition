package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;

public class Elevator extends SubsystemBase{
    
    private TalonFX m_rightElevatorMotor;
    private TalonFX m_leftElevatorMotor;

    private double m_targetHeight;

    public Elevator(){
        m_rightElevatorMotor = new TalonFX(Ports.RIGHT_ELEVATORMOTOR_ID, Ports.CAN_BUS_NAME);
        m_leftElevatorMotor = new TalonFX(Ports.LEFT_ELEVATORMOTOR_ID, Ports.CAN_BUS_NAME);

        configShooterMotors();

        m_leftElevatorMotor.setControl(new Follower(m_rightElevatorMotor.getDeviceID(), true)); //leftShooterMotor follows rightShooterMotor in the opposing direction
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("rightElevatorMotor", m_rightElevatorMotor, null, Constants.LOOP_TIME_HZ); //TODO create elevator config
        DeviceConfig.configureTalonFX("leftElevatorMotor", m_leftElevatorMotor, null, Constants.LOOP_TIME_HZ); //TODO create elevator config
    }

    public void setNeutralModes(NeutralModeValue elevatorMode){
        m_rightElevatorMotor.setNeutralMode(elevatorMode);
        m_leftElevatorMotor.setNeutralMode(elevatorMode);
    }

    public void zeroHeight(){
        m_rightElevatorMotor.setPosition(0.0);
    }

    public void toTargetHeight(double height){
        m_targetHeight = height;

        if(!isPivotAtTarget()) {
            setElevator(new MotionMagicVoltage(m_targetHeight));
        } else {
            stopElevator();
        }
    }
    
    public void setElevator(ControlRequest control){
        m_rightElevatorMotor.setControl(control);
    }

    public void setElevator_Manual(double output){
        m_rightElevatorMotor.set(output);
    }

    public void stopElevator(){
        m_rightElevatorMotor.stopMotor();
    }

    public double getElevatorHeight(){
        return m_rightElevatorMotor.getPosition().refresh().getValue();
    }

    public double getPivotError(){
        return m_targetHeight - getElevatorHeight();
    }

    public boolean isPivotAtTarget(){
        return Math.abs(getPivotError()) < ElevatorConstants.TARGET_THRESHOLD;
    }
    
    public double getElevatorHeight(double distance){
        return ElevatorConstants.elevatorTreeMap.get(distance);
    }

    @Override
    public void periodic() {

    }
}
