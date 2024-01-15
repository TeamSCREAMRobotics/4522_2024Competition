package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class Conveyor extends SubsystemBase{
        
    private TalonFX m_conveyorMotor;

    public Conveyor(){
        m_conveyorMotor = new TalonFX(Ports.CONVEYOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configShooterMotors();
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Conveyor Motor", m_conveyorMotor, new TalonFXConfiguration(), Constants.LOOP_TIME_HZ); //TODO create conveyor config
    }
    
    public void setNeutralModes(NeutralModeValue mode){
        m_conveyorMotor.setNeutralMode(mode);
    }

    public void setConveyorSpeed(double output){
        m_conveyorMotor.set(output);
    }

    public void stopMotor(){
        m_conveyorMotor.stopMotor();
    }

    @Override
    public void periodic(){
        
    }
}
