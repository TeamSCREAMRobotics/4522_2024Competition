package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.lib.util.OrchestraUtil;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase{
    TalonFX m_leftIntakeMotor;
    TalonFX m_rightIntakeMotor;

    public Intake(){
        // m_leftIntakeMotor = new TalonFX(Ports.LEFT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        // m_rightIntakeMotor = new TalonFX(Ports.RIGHT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        //configIntakeMotors();

        //OrchestraUtil.add(m_leftIntakeMotor, m_rightIntakeMotor);
    }

    public void configIntakeMotors(){
        // DeviceConfig.configureTalonFX("Left Intake Motor", m_leftIntakeMotor, DeviceConfig.intakeFXConfig(), Constants.LOOP_TIME_HZ);
        // DeviceConfig.configureTalonFX("Right Intake Motor", m_rightIntakeMotor, DeviceConfig.intakeFXConfig(), Constants.LOOP_TIME_HZ);
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_leftIntakeMotor.setNeutralMode(mode);
        m_rightIntakeMotor.setNeutralMode(mode);
    }

    public void setIntakeOutput(double po){
        setIntake(new DutyCycleOut(po));
    }

    public void setIntake(ControlRequest control){
        m_rightIntakeMotor.setControl(control);
        m_leftIntakeMotor.setControl(new Follower(m_rightIntakeMotor.getDeviceID(), false));
    }

    public void stop(){
        m_rightIntakeMotor.stopMotor();
        m_leftIntakeMotor.stopMotor();
    }
}
