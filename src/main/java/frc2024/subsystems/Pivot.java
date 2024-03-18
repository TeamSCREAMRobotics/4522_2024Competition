package frc2024.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.RunOnce;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.StabilizerConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;
import frc2024.controlboard.Controlboard;

public class Pivot extends SubsystemBase{
    
    private TalonFX m_pivotMotor;
    private CANcoder m_encoder;

    private MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);
    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);
    private Rotation2d m_tweakAngle = Rotation2d.fromDegrees(0);

    public Pivot(){
        m_pivotMotor = new TalonFX(Ports.PIVOT_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_encoder = new CANcoder(Ports.PIVOT_ENCODER_ID, Ports.RIO_CANBUS_NAME);
         
        configureDevices(PivotConstants.SOFTWARE_LIMIT_ENABLE, PivotConstants.FORWARD_SOFT_LIMIT_ENDGAME, PivotConstants.REVERSE_SOFT_LIMIT_ENDGAME);
        
        OrchestraUtil.add(m_pivotMotor);
    }

    public void configureDevices(boolean softwareLimitEnable, Rotation2d forwardLimit, Rotation2d reverselimit) {
        DeviceConfig.configureCANcoder("Pivot Encoder", m_encoder, DeviceConfig.pivotEncoderConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Pivot Motor", m_pivotMotor, DeviceConfig.pivotFXConfig(softwareLimitEnable, forwardLimit, reverselimit), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_pivotMotor, m_encoder);
    }

    public void configPID(ScreamPIDConstants constants){
        m_pivotMotor.getConfigurator().apply(constants.toSlot0Configs(PivotConstants.FEEDFORWARD_CONSTANTS));
    }

    public void zeroPivot(){
        m_pivotMotor.setPosition(0.0);
        m_encoder.setPosition(0.0);
    }

    public void resetToAbsolute(){
        m_pivotMotor.setPosition(m_encoder.getPosition().getValueAsDouble());
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_pivotMotor.setNeutralMode(mode);
    }

    public void setPivot(ControlRequest control){
        m_pivotMotor.setControl(control);
    }

    public void setTargetAngle(Rotation2d angle){
        m_targetAngle = angle;
        setPivot(m_positionRequest.withPosition(m_targetAngle.getRotations()));
    }

    public void setPivotOutput(double output){
        setPivot(m_dutyCycleRequest.withOutput(output)
            .withLimitForwardMotion(RobotContainer.forwardPivotLimit())
            .withLimitReverseMotion(RobotContainer.reversePivotLimit()));
    }

    public Rotation2d getPivotAngle(){
        return Rotation2d.fromRotations(m_pivotMotor.getPosition().refresh().getValue());
    }

    public Rotation2d getPivotError(){
        return Rotation2d.fromDegrees(Math.abs(m_targetAngle.getDegrees()) - Math.abs(getPivotAngle().getDegrees()));
    }

    public BooleanSupplier getPivotAtTarget(){
        return () -> Math.abs(getPivotError().getDegrees()) < PivotConstants.TARGET_THRESHOLD;
    }

    public double getPivotCurrent(){
        return m_pivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    public void stop(){
        m_pivotMotor.stopMotor();
    }

    @Override
    public void periodic() {
        //System.out.println("Pivot:" + m_targetAngle.getDegrees());
        //logOutputs();
        /* if(Controlboard.increasePivot().getAsBoolean()){
            m_tweakAngle = m_tweakAngle.minus(Rotation2d.fromDegrees(1));
        } else if(Controlboard.decreasePivot().getAsBoolean()){
            m_tweakAngle = m_tweakAngle.plus(Rotation2d.fromDegrees(1));
        }
        System.out.println(Controlboard.operatorController_Command.ge); */
        //System.out.println("Encoder: " + getPivotAngle().getDegrees());
        //System.out.println("Motor: " + Rotation2d.fromRotations(m_pivotMotor.getPosition().refresh().getValue()).getDegrees());
    }

    public void logOutputs(){
        Logger.recordOutput("Pivot/Measured/Angle", getPivotAngle());
        Logger.recordOutput("Pivot/Measured/Rotations", m_pivotMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Pivot/Measured/ErrorDegrees", getPivotError());
        Logger.recordOutput("Pivot/Setpoint/Angle", m_targetAngle);
        Logger.recordOutput("Pivot/Setpoint/Rotations", m_targetAngle.getRotations());
        Logger.recordOutput("Pivot/Setpoint/AtTarget", getPivotAtTarget().getAsBoolean());
        Logger.recordOutput("Pivot/Power/StatorCurrent", m_pivotMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Pivot/Power/SupplyCurrent", m_pivotMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Pivot/Power/Voltage", m_pivotMotor.getSupplyVoltage().getValueAsDouble());
        //Logger.recordOutput("Pivot/CurrentCommand", getCurrentCommand().getName());
    }

    public Command dutyCycleCommand(DoubleSupplier dutyCycle){
        return run(() -> setPivotOutput(dutyCycle.getAsDouble())).withName("DutyCycleCommand[Supplier]");
    }

    public Command dutyCycleCommand(double dutyCycle){
        return run(() -> setPivotOutput(dutyCycle)).withName("DutyCycleCommand");
    }

    public Command angleCommand(Rotation2d angle){
        return run(() -> setTargetAngle(angle.plus(m_tweakAngle))).withName("AngleCommand");
    }

    public Command angleCommand(Supplier<Rotation2d> angle){
        return run(() -> setTargetAngle(angle.get())).withName("AngleCommand[Supplier]");
    }
}
