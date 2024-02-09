package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.Ports;
import frc2024.Constants.SwerveConstants;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class Intake extends SubsystemBase{
    private TalonFX m_leftIntakeMotor;
    private TalonFX m_rightIntakeMotor;

    public Intake(){
        m_leftIntakeMotor = new TalonFX(Ports.LEFT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_rightIntakeMotor = new TalonFX(Ports.RIGHT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        configIntakeMotors();

        m_leftIntakeMotor.setControl(new Follower(m_rightIntakeMotor.getDeviceID(), false));
        //OrchestraUtil.add(m_leftIntakeMotor, m_rightIntakeMotor);
    }

    public void configIntakeMotors(){
        DeviceConfig.configureTalonFX("Left Intake Motor", m_leftIntakeMotor, DeviceConfig.intakeFXConfig(), Constants.LOOP_TIME_HZ);
        DeviceConfig.configureTalonFX("Right Intake Motor", m_rightIntakeMotor, DeviceConfig.intakeFXConfig(), Constants.LOOP_TIME_HZ);
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_leftIntakeMotor.setNeutralMode(mode);
        m_rightIntakeMotor.setNeutralMode(mode);
    }

    public double getMotorRPM(){
        return ScreamUtil.average(m_rightIntakeMotor.getVelocity().getValueAsDouble(), m_leftIntakeMotor.getVelocity().getValueAsDouble())*60;
    }

    public void setIntakeOutput(double po){
        setIntake(new DutyCycleOut(po));
    }

    public void setIntake(ControlRequest control){
        m_rightIntakeMotor.setControl(control);
    }

    public void stop(){
        m_rightIntakeMotor.stopMotor();
        m_leftIntakeMotor.stopMotor();
    }

    public Command outputCommand(double output){
        return run(() -> setIntakeOutput(output));
    }

    public Command stopCommand(){
        return run(() -> stop());
    }
}
