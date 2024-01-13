package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class Conveyor extends SubsystemBase{
        
    private TalonFX m_conveyorMotor;

    public Conveyor(){
        m_conveyorMotor = new TalonFX(Ports.CONVEYORMOTOR_ID, Ports.CAN_BUS_NAME);

        configShooterMotors();
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("conveyorMotor", m_conveyorMotor, null, Constants.LOOP_TIME_HZ); //TODO create conveyor config
    }
    
    public void setNeutralModes(NeutralModeValue conveyorMode){
        m_conveyorMotor.setNeutralMode(conveyorMode);
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
