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
    private TalonFX m_intakeMotor;

    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    public Intake(){
        m_intakeMotor = new TalonFX(Ports.RIGHT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        configIntakeMotors();

        OrchestraUtil.add(m_intakeMotor);
    }

    public void configIntakeMotors(){
        DeviceConfig.configureTalonFX("Right Intake Motor", m_intakeMotor, DeviceConfig.intakeFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_intakeMotor.setNeutralMode(mode);
    }

    public void setIntakeOutput(double output){
        setIntake(m_dutyCycleRequest.withOutput(output));
    }

    public void setIntake(ControlRequest control){
        m_intakeMotor.setControl(control);
    }

    public double getRPM(){
        return m_intakeMotor.getVelocity().getValueAsDouble()*60.0;
    }

    public void stop(){
        m_intakeMotor.stopMotor();
    }

    public Command outputCommand(double output){
        return run(() -> setIntakeOutput(output));
    }

    public Command stopCommand(){
        return run(() -> stop());
    }
}
