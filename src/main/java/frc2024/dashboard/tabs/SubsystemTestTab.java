// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.dashboard.tabs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.ShuffleboardConstants;
import frc2024.dashboard.ShuffleboardTabBase;
import frc2024.subsystems.Climber;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;

/** Add your docs here. */
public class SubsystemTestTab extends ShuffleboardTabBase{

    private Climber climber;
    private Conveyor conveyor;
    private Elevator elevator;
    private Intake intake;
    private Pivot pivot;
    private Shooter shooter;

    public SubsystemTestTab(Climber climber, Conveyor conveyor, Elevator elevator, Intake intake, Pivot pivot, Shooter shooter){
        this.climber = climber;
        this.conveyor = conveyor;
        this.elevator = elevator;
        this.intake = intake;
        this.pivot = pivot;
        this.shooter = shooter;
    }

    /* Climber */
    private static GenericEntry m_climberHeight_Target;
    private GenericEntry m_climberHeight;
    private static GenericEntry m_climberKP;

    /* Conveyor */
    private static GenericEntry m_conveyor_DutyCycle;
    private GenericEntry m_conveyor_MotorVelocity;

    /* Elevator */
    private static GenericEntry m_elevatorHeight_Target;
    private GenericEntry m_elevatorHeight;
    private static GenericEntry m_elevatorKP;

    /* Intake */
    private static GenericEntry m_intake_DutyCycle;
    private GenericEntry m_intake_MotorVelocity;

    /* Pivot */
    private static GenericEntry m_pivotAngle_Target;
    private GenericEntry m_pivotAngle;
    private static GenericEntry m_pivotKP;
        
    /* Shooter */
    private static GenericEntry m_shooter_DutyCycle;
    private static GenericEntry m_shooter_Velocity;
    private GenericEntry m_shooter_MotorVelocity;
    private GenericEntry m_shooter_WheelVelocity;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Subsytem Testing");

        /* Climber */
        m_climberHeight = createNumberEntry("Climber Height", 0, new EntryProperties(0, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_CLIMBER){
            m_climberHeight_Target = createNumberEntry("Climber Target Height", 0.0, new EntryProperties(0, 1, 2, 1));
        }

        /* Conveyor */
        m_conveyor_MotorVelocity = createNumberEntry("Conveyor Motor RPM", 0, new EntryProperties(1, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_CONVEYOR){
            m_conveyor_DutyCycle = createNumberEntry("Climber Target Duty Cycle", 0.0, new EntryProperties(1, 1, 2, 1));
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
        m_shooter_WheelVelocity = createNumberEntry("Shooter Wheel RPM", 0, new EntryProperties(6, 0, 2, 1));

        if(ShuffleboardConstants.UPDATE_SHOOTER){
            m_shooter_DutyCycle = createNumberEntry("Shooter Target Duty Cycle", 0.0, new EntryProperties(5, 1, 2, 1));
            m_shooter_Velocity = createNumberEntry("Shooter Target Velocity", 0.0, new EntryProperties(6, 1, 2, 1));
        }
    }
    
    @Override
    public void periodic() {
        /* Climber */
        if(ShuffleboardConstants.UPDATE_CLIMBER){
            m_climberHeight.setDouble(climber.getClimberHeight());

            climber.configPID(ClimberConstants.PID_CONSTANTS.withP(m_climberKP.get().getDouble()));
            climber.setTargetPosition(getClimberHeight());
        }

        /* Conveyor */
        if(ShuffleboardConstants.UPDATE_CONVEYOR){
            m_conveyor_MotorVelocity.setDouble(conveyor.getMotorRPM());
            conveyor.setConveyorOutput(getConveyorDutyCycle());
        }

        /* Elevator */
        if(ShuffleboardConstants.UPDATE_ELEVATOR){
            m_elevatorHeight.setDouble(elevator.getElevatorHeight());

            elevator.configPID(ClimberConstants.PID_CONSTANTS.withP(m_elevatorKP.get().getDouble()));
            elevator.setTargetPosition(getElevatorHeight());
        }

        /* Intake */
        if(ShuffleboardConstants.UPDATE_INTAKE){
            m_intake_MotorVelocity.setDouble(intake.getMotorRPM());

            intake.setIntakeOutput(getIntakeDutyCycle());
        }

        /* Pivot */
        if(ShuffleboardConstants.UPDATE_PIVOT){
            m_pivotAngle.setDouble(pivot.getPivotAngle().getDegrees());

            pivot.configPID(ClimberConstants.PID_CONSTANTS.withP(m_pivotKP.get().getDouble()));
            pivot.setTargetAngle(new Rotation2d(getPivotAngle()));
        }

        /* Shooter */
        if(ShuffleboardConstants.UPDATE_SHOOTER){
            m_shooter_MotorVelocity.setDouble(shooter.getMotorRPM());
            m_shooter_WheelVelocity.setDouble(shooter.getWheelRPM());

            shooter.setShooterOutput(getShooterDutyCycle());
            shooter.setTargetVelocity(getShooterVelocity());
        }
    }

    /* Climber */
    public static double getClimberHeight(){
        return m_climberHeight_Target.getDouble(0);
    }

    /* Conveyor */
    public static double getConveyorDutyCycle(){
        return m_conveyor_DutyCycle.getDouble(0);
    }

    /* Elevator */
    public static double getElevatorHeight(){
        return m_elevatorHeight_Target.getDouble(0);
    }

    /* Intake */
    public static double getIntakeDutyCycle(){
        return m_intake_DutyCycle.getDouble(0);
    }

    /* Pivot */
    public static double getPivotAngle(){
        return m_pivotAngle_Target.getDouble(0);
    }

    /* Shooter */
    public static double getShooterDutyCycle(){
        return m_shooter_DutyCycle.getDouble(0);
    }
    public static double getShooterVelocity(){
        return m_shooter_Velocity.getDouble(0);
    }
}
