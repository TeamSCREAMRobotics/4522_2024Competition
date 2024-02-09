package frc2024.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.Ports;
import frc2024.Constants.ShooterConstants;
import frc2024.dashboard.tabs.SubsystemTestTab;

public class Shooter extends SubsystemBase{
    
    private TalonFX m_rightShooterMotor;
    private TalonFX m_leftShooterMotor;

    public Shooter(){
        m_rightShooterMotor = new TalonFX(Ports.RIGHT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_leftShooterMotor = new TalonFX(Ports.LEFT_SHOOTER_MOTOR_ID, Ports.RIO_CANBUS_NAME);

        configShooterMotors();
        
        //OrchestraUtil.add(m_rightShooterMotor, m_leftShooterMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("rightShooterMotor", m_rightShooterMotor, DeviceConfig.shooterFXConfig(), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("leftShooterMotor", m_leftShooterMotor, DeviceConfig.shooterFXConfig(), Constants.LOOP_TIME_HZ);
     }
    
    public void setNeutralMode(NeutralModeValue shooterMode){
        m_rightShooterMotor.setNeutralMode(shooterMode);
        m_leftShooterMotor.setNeutralMode(shooterMode);
    }

    public void setTargetVelocity(double rpm){
        m_rightShooterMotor.setControl(new VelocityVoltage(Conversions.rpmToFalconRPS(rpm, 1.0)));
    }

    public double getWheelRPM(){
        return Conversions.falconRPSToMechanismRPM(ScreamUtil.average(m_rightShooterMotor.getVelocity().getValueAsDouble(), m_leftShooterMotor.getVelocity().getValueAsDouble()), 1.0);
    }

    public double getMotorRPM(){
        return ScreamUtil.average(m_rightShooterMotor.getVelocity().getValueAsDouble(), m_leftShooterMotor.getVelocity().getValueAsDouble())*60;
    }

    public void setShooterOutput(double po){
        setShooter(new DutyCycleOut(po));
    }

    public void setShooter(ControlRequest control){
        m_rightShooterMotor.setControl(control);
        m_leftShooterMotor.setControl(new Follower(m_rightShooterMotor.getDeviceID(), false)); //left motor follows right motor in the opposing direction
    }

    public void stop(){
        m_rightShooterMotor.stopMotor();
        m_leftShooterMotor.stopMotor();
    }

    public Command outputCommand(double output){
        return run(() -> setShooterOutput(output));
    }

    public Command velocityCommand(double velocity){
        return run(() -> setTargetVelocity(velocity));
    }

    public Command stopCommand(){
        return run(() -> stop());
    }
}
