package frc2024.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.Ports;
import frc2024.Constants.RobotMode;

public class Conveyor extends SubsystemBase{
        
    private TalonFX m_conveyorMotor;
    private DigitalInput m_beam;

    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
    private Debouncer m_beamDebouncer = new Debouncer(0.14, DebounceType.kBoth);

    public Conveyor(){
        m_conveyorMotor = new TalonFX(Ports.CONVEYOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_beam = new DigitalInput(Ports.CONVEYOR_BEAM_ID);

        configShooterMotors();
        
        OrchestraUtil.add(m_conveyorMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Conveyor Motor", m_conveyorMotor, DeviceConfig.conveyorFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_conveyorMotor);
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
    
    public BooleanSupplier hasPiece(boolean trapBeam){
        return trapBeam ? () -> m_beamDebouncer.calculate(!m_beam.get()) : () -> !m_beam.get();
    }

    public void stop(){
        m_conveyorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // System.out.println(!m_beam.get());
        if(Constants.MODE == RobotMode.COMP){
            logOutputs();
        }
    }

    public void logOutputs(){
        ScreamUtil.logBasicMotorOutputs("Conveyor", m_conveyorMotor);
    }

    public Command dutyCycleCommand(double output){
        return run(() -> setConveyorOutput(output));
    }

    public Command stopCommand(){
        return Commands.runOnce(() -> stop(), this);
    }

    public Command scoreCommand(){
        return new ConditionalCommand(
            dutyCycleCommand(ConveyorConstants.AMP_OUTPUT), 
            new ConditionalCommand(
                dutyCycleCommand(ConveyorConstants.SHOOT_OUTPUT), 
                stopCommand(), 
                () -> RobotContainer.getCurrentState().get() != SuperstructureState.HOME
                && RobotContainer.getCurrentState().get() != SuperstructureState.HOME_ENDGAME
                && RobotContainer.getCurrentState().get() != SuperstructureState.NONE), 
            () -> RobotContainer.getCurrentState().get() == SuperstructureState.AMP);
    }
}
