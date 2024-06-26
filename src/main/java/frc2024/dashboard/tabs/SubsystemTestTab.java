// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.dashboard.tabs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc2024.Constants.ShuffleboardConstants;
import frc2024.dashboard.ShuffleboardTabBase;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;

/** Add your docs here. */
public class SubsystemTestTab extends ShuffleboardTabBase{

    private Conveyor conveyor;
    private Elevator elevator;
    private Intake intake;
    private Pivot pivot;
    private Shooter shooter;

    public SubsystemTestTab(Conveyor conveyor, Elevator elevator, Intake intake, Pivot pivot, Shooter shooter){
        this.conveyor = conveyor;
        this.elevator = elevator;
        this.intake = intake;
        this.pivot = pivot;
        this.shooter = shooter;
    }

    /* Conveyor */
    private static GenericEntry m_conveyor_DutyCycle;
    private GenericEntry m_conveyor_MotorVelocity;

    /* Elevator */
    private static GenericEntry m_elevatorHeight_Target;
    private GenericEntry m_elevatorHeight;

    /* Intake */
    private static GenericEntry m_intake_DutyCycle;
    private GenericEntry m_intake_MotorVelocity;

    /* Pivot */
    private static GenericEntry m_pivotAngle_Target;
    private GenericEntry m_pivotAngle;
        
    /* Shooter */
    private static GenericEntry m_shooter_Velocity;
    private GenericEntry m_shooter_MotorVelocity;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Subsytem Testing");

        /* Conveyor */
        m_conveyor_MotorVelocity = createNumberEntry("Conveyor Motor RPM", 0, new EntryProperties(1, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_CONVEYOR){
            m_conveyor_DutyCycle = createNumberEntry("Conveyor Target Duty Cycle", 0.0, new EntryProperties(1, 1, 2, 1));
        }

        /* Elevator */
        m_elevatorHeight = createNumberEntry("Elevator Height", 0, new EntryProperties(2, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_ELEVATOR){
            m_elevatorHeight_Target = createNumberEntry("Elevator Target Height", 0.0, new EntryProperties(2, 1, 2, 1));
        }

        /* Intake */
        m_intake_MotorVelocity = createNumberEntry("Intake Motor RPM", 0, new EntryProperties(3, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_INTAKE){
            m_intake_DutyCycle = createNumberEntry("Intake Target Duty Cycle", 0.0, new EntryProperties(3, 1, 2, 1));
        }

        /* Pivot */
        m_pivotAngle = createNumberEntry("Pivot Angle", 0, new EntryProperties(4, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_PIVOT){
            m_pivotAngle_Target = createNumberEntry("Pivot Target Angle", 0.0, new EntryProperties(4, 1, 2, 1));
        }
        
        /* Shooter */
        m_shooter_MotorVelocity = createNumberEntry("Shooter Motor RPM", 0, new EntryProperties(5, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_SHOOTER){
            m_shooter_Velocity = createNumberEntry("Shooter Target Velocity", 0.0, new EntryProperties(6, 1, 2, 1));
        }
    }
    
    @Override
    public void periodic() {
        /* Conveyor */
        if(ShuffleboardConstants.UPDATE_CONVEYOR){
            conveyor.setConveyorOutput(getConveyorDutyCycle());
        }

        /* Elevator */
        if(ShuffleboardConstants.UPDATE_ELEVATOR){
            elevator.setTargetHeight(getElevatorHeight());
        }

        /* Intake */
        if(ShuffleboardConstants.UPDATE_INTAKE){
            intake.setIntakeOutput(getIntakeDutyCycle());
        }

        /* Pivot */
        if(ShuffleboardConstants.UPDATE_PIVOT){
            pivot.setTargetAngle(new Rotation2d(getPivotAngle()));
        }

        /* Shooter */
        if(ShuffleboardConstants.UPDATE_SHOOTER){
            shooter.setTargetVelocity(getShooterVelocity());
        }

        m_shooter_MotorVelocity.setDouble(shooter.getRPM());
        m_pivotAngle.setDouble(pivot.getPivotAngle().getDegrees());
        m_intake_MotorVelocity.setDouble(intake.getRPM());
        m_elevatorHeight.setDouble(elevator.getElevatorPosition());
        m_conveyor_MotorVelocity.setDouble(conveyor.getRPM());

    }

    /* Conveyor */
    public double getConveyorDutyCycle(){
        return m_conveyor_DutyCycle.getDouble(0);
    }

    /* Elevator */
    public double getElevatorHeight(){
        return m_elevatorHeight_Target.getDouble(elevator.getElevatorHeight());
    }

    /* Intake */
    public double getIntakeDutyCycle(){
        return m_intake_DutyCycle.getDouble(0);
    }

    /* Pivot */
    public double getPivotAngle(){
        return m_pivotAngle_Target.getDouble(pivot.getPivotAngle().getRadians());
    }

    /* Shooter */
    public double getShooterVelocity(){
        return m_shooter_Velocity.getDouble(0);
    }
}
