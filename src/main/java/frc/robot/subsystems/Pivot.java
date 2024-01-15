package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Ports;

public class Pivot extends SubsystemBase{
    
    private TalonFX m_pivotMotor;

    private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

    public Pivot(){
        //m_pivotMotor = new TalonFX(Ports.PIVOT_MOTOR_ID);
        
        configPivotMotor();
    }

    private void configPivotMotor() {
        // DeviceConfig.configureTalonFX("pivotMotor", m_pivotMotor, DeviceConfig.pivotFXConfig(), Constants.LOOP_TIME_HZ);
    }
    
    public void setNeutralModes(NeutralModeValue mode){
        m_pivotMotor.setNeutralMode(mode);
    }

    public void zeroPivot(){
        m_pivotMotor.setPosition(0.0);
    }

    public void pivotToTargetAngle(Rotation2d angle){
        m_targetAngle = angle;

        if(!pivotAtTarget()) {
            setPivot(new MotionMagicVoltage(m_targetAngle.getRotations()));
        } else {
            stopPivot();
        }
    }
    
    public void setPivot(ControlRequest control){
        m_pivotMotor.setControl(control);
    }

    public void stopPivot(){
        m_pivotMotor.stopMotor();
    }

    public Rotation2d getPivotAngle(){
        return Rotation2d.fromRotations(m_pivotMotor.getPosition().refresh().getValue());
    }

    public Rotation2d getPivotError(){
        return m_targetAngle.minus(getPivotAngle());
    }

    public boolean pivotAtTarget(){
        return Math.abs(getPivotError().getDegrees()) < PivotConstants.TARGET_THRESHOLD;
    }

    public double getPivotTargetAngle(double distance){
        return PivotConstants.pivotTreeMap.get(distance);
    }

    @Override
    public void periodic() {

    }
}
