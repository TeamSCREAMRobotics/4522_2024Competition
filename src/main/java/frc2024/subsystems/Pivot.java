package frc2024.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;
import frc2024.Constants.RobotMode;
import frc2024.controlboard.Controlboard;

public class Pivot extends SubsystemBase{
    
    private TalonFX m_pivotMotor;
    private CANcoder m_encoder;

    private PositionVoltage m_positionRequest = new PositionVoltage(0);
    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);
    private Rotation2d m_tweakAngle = Rotation2d.fromDegrees(0);

    public Pivot(){
        m_pivotMotor = new TalonFX(Ports.PIVOT_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_encoder = new CANcoder(Ports.PIVOT_ENCODER_ID, Ports.RIO_CANBUS_NAME);
         
        configureDevices(PivotConstants.SOFTWARE_LIMIT_ENABLE, PivotConstants.FORWARD_SOFT_LIMIT_ENDGAME, PivotConstants.REVERSE_SOFT_LIMIT_ENDGAME);

        m_pivotMotor.getConfigurator().apply(PivotConstants.PID_CONSTANTS.toSlot0Configs(PivotConstants.FEEDFORWARD_CONSTANTS));
        
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

    public void configCurrentLimit(CurrentLimitsConfigs config){
        m_pivotMotor.getConfigurator().apply(config);
    }

    public void zeroPivot(){
        m_pivotMotor.setPosition(0.0);
        m_encoder.setPosition(0.0);
    }

    public void resetToAbsolute(){
        m_pivotMotor.setPosition(m_encoder.getAbsolutePosition().refresh().getValueAsDouble());
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_pivotMotor.setNeutralMode(mode);
    }

    public void setPivot(ControlRequest control){
        if(m_encoder.getSupplyVoltage().getValueAsDouble() == 0.0){
            DriverStation.reportError("Encoder Power Bad", true);
            m_pivotMotor.disable();
        } else {
            m_pivotMotor.setControl(control);
        }
    }

    public void setTargetAngle(Rotation2d angle){
        m_targetAngle = angle;
        if(!getPivotAtTarget().getAsBoolean()){
            setPivot(m_positionRequest.withPosition(m_targetAngle.getRotations()));
        } else {
            stop();
        }
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
        if(Controlboard.operatorController_Command.getHID().getRightStickButtonPressed()){
            m_tweakAngle = m_tweakAngle.minus(Rotation2d.fromDegrees(0.5));
        } else if(Controlboard.operatorController_Command.getHID().getLeftStickButtonPressed()){
            m_tweakAngle = m_tweakAngle.plus(Rotation2d.fromDegrees(0.5));
        }
        if(Constants.MODE == RobotMode.COMP){
            logOutputs();
        }
    }

    public void logOutputs(){
        ScreamUtil.logBasicMotorOutputs("Pivot", m_pivotMotor);
        ScreamUtil.logServoMotorOutputs("Pivot", m_pivotMotor);
        Logger.recordOutput("Pivot/Measured/Angle", getPivotAngle());
        Logger.recordOutput("Pivot/Measured/AngleHorizontal", getPivotAngle().unaryMinus().plus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL));
        Logger.recordOutput("Pivot/Measured/Error", getPivotError());
        Logger.recordOutput("Pivot/Setpoint/Angle", m_targetAngle);
        Logger.recordOutput("Pivot/Setpoint/AtTarget", getPivotAtTarget().getAsBoolean());
        if(getCurrentCommand() != null){
            Logger.recordOutput("Pivot/CurrentCommand", getCurrentCommand().getName());
        }
    }

    public Command dutyCycleCommand(DoubleSupplier dutyCycle){
        return run(() -> setPivotOutput(dutyCycle.getAsDouble())).withName("DutyCycleCommand");
    }

    public Command dutyCycleCommand(double dutyCycle){
        return dutyCycleCommand(() -> dutyCycle);
    }

    public Command angleCommand(Supplier<Rotation2d> angle){
        return run(() -> setTargetAngle(angle.get().plus(m_tweakAngle))).withName("AngleCommand");
    }

    public Command angleCommand(Rotation2d angle){
        return angleCommand(() -> angle);
    }
}
