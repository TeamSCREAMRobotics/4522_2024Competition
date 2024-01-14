package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase{
    TalonFX m_leftIntakeMotor;
    TalonFX m_rightIntakeMotor;

    public Intake(){
        m_leftIntakeMotor = new TalonFX(Ports.LEFT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_rightIntakeMotor = new TalonFX(Ports.RIGHT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        configIntakeMotors();

        m_rightIntakeMotor.setControl(new Follower(Ports.LEFT_INTAKE_MOTOR_ID, false));
    }

    public void configIntakeMotors(){
        DeviceConfig.configureTalonFX("Left Intake Motor", m_leftIntakeMotor, new TalonFXConfiguration(), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Right Intake Motor", m_rightIntakeMotor, new TalonFXConfiguration(), Constants.LOOP_TIME_HZ);
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_leftIntakeMotor.setNeutralMode(mode);
        m_rightIntakeMotor.setNeutralMode(mode);
    }

    public void setIntake(ControlRequest control){
        m_leftIntakeMotor.setControl(control);
    }

    public void stopIntake(){
        m_leftIntakeMotor.stopMotor();
    }
}
