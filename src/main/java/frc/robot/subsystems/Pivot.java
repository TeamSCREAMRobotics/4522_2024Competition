package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.lib.util.OrchestraUtil;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Ports;

public class Pivot extends SubsystemBase{
    
    private TalonFX m_pivotMotor;
    private DutyCycleEncoder m_encoder;

    private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

    public Pivot(){
        //m_pivotMotor = new TalonFX(Ports.PIVOT_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        //m_encoder = new DutyCycleEncoder(Ports.PIVOT_ENCODER_ID);
        
        configPivotMotor();
        // resetPivotToAbsoulute();
        
        //OrchestraUtil.add(m_pivotMotor);
    }

    private void configPivotMotor() {
        // DeviceConfig.configureTalonFX("pivotMotor", m_pivotMotor, DeviceConfig.pivotFXConfig(), Constants.LOOP_TIME_HZ);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_pivotMotor.setNeutralMode(mode);
    }

    public void resetPivotToAbsoulute(){
        m_pivotMotor.setPosition(m_encoder.getAbsolutePosition());
    }

    public void zeroPivot(){
        m_pivotMotor.setPosition(0.0);
    }

    public void setTargetAngle(Rotation2d angle){
        m_targetAngle = angle;
        setPivot(new MotionMagicVoltage(angle.getRotations()));
    }
    
    public void setPivot(ControlRequest control){
        m_pivotMotor.setControl(control);
    }

    public Rotation2d getPivotAngle(){
        return Rotation2d.fromRotations(m_pivotMotor.getPosition().refresh().getValue());
    }

    public Rotation2d getPivotError(){
        return m_targetAngle.minus(getPivotAngle());
    }

    public boolean getPivotAtTarget(){
        return Math.abs(getPivotError().getDegrees()) < PivotConstants.TARGET_THRESHOLD;
    }

    public double getPivotTargetAngle(double distance){
        return PivotConstants.pivotTreeMap.get(distance);
    }

    public void stop(){
        m_pivotMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // resetPivotToAbsoulute(); //Do we want the motor's position to be constantly reset during the match?
    }
}
