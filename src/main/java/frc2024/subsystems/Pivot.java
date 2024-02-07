package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;

public class Pivot extends SubsystemBase{
    
    private TalonFX m_pivotMotor;
    private DutyCycleEncoder m_encoder;

    private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

    private ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0);

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

    public void setPivotOutput(double po){
        setPivot(new DutyCycleOut(po));
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
    
    public double getPivotTargetAngle_Localization(double distance){
        return PivotConstants.ANGLE_MAP_UNDEFENDED.get(distance);
    }

    public void stop(){
        m_pivotMotor.stopMotor();
    }

    public Command outputCommand(DoubleSupplier output){
        return run(() -> setPivotOutput(output.getAsDouble()));
    }

    public Command outputCommand(double output){
        return run(() -> setPivotOutput(output));
    }

    public Command angleCommand(Rotation2d angle){
        return run(() -> setTargetAngle(angle));
    }
}
