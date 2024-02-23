package frc2024.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.Ports;

public class Conveyor extends SubsystemBase{
        
    private TalonFX m_conveyorMotor;
    private DigitalInput m_beam;

    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
    private Debouncer m_beamDebouncer = new Debouncer(0.05, DebounceType.kBoth);

    public Conveyor(){
        m_conveyorMotor = new TalonFX(Ports.CONVEYOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_beam = new DigitalInput(Ports.CONVEYOR_BEAM_ID);

        configShooterMotors();
        
        OrchestraUtil.add(m_conveyorMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Conveyor Motor", m_conveyorMotor, DeviceConfig.conveyorFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_conveyorMotor.setNeutralMode(mode);
    }

    public void setConveyor(ControlRequest control){
        m_conveyorMotor.setControl(control);
    }

    public void setConveyorOutput(double output){
        setConveyor(m_dutyCycleRequest.withOutput(output));
    }

    public double getRPM(){
        return m_conveyorMotor.getVelocity().getValueAsDouble()*60;
    }
    
    public BooleanSupplier hasPiece(){
        return () -> m_beamDebouncer.calculate(!m_beam.get());
    }

    public void stop(){
        m_conveyorMotor.stopMotor();
    }

    @Override
    public void periodic() {}

    public Command outputCommand(double output){
        return run(() -> setConveyorOutput(output));
    }

    public Command stopCommand(){
        return run(() -> stop());
    }


}
