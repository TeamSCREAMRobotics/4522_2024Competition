// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dashboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.dashboard.ShuffleboardTabBase;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class ShooterTab extends ShuffleboardTabBase{

    private Shooter shooter;

    public ShooterTab(Shooter shooter){
        this.shooter = shooter;
    }

    private static GenericEntry m_dutyCycle;
    private static GenericEntry m_velocity;

    private GenericEntry m_motorVelocity;
    private GenericEntry m_wheelVelocity;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Shooter");

        m_motorVelocity = createNumberEntry("Motor RPM", 0, new EntryProperties(2, 0, 2, 1));
        m_wheelVelocity = createNumberEntry("Wheel RPM", 0, new EntryProperties(2, 1, 2, 1));

        if(ShuffleboardConstants.UPDATE_SHOOTER){
            m_dutyCycle = createNumberEntry("Shooter Target Duty Cycle", 0.0, new EntryProperties(0, 0, 2, 1));
            m_velocity = createNumberEntry("Shooter Target Velocity", 0.0, new EntryProperties(0, 1, 2, 1));
        }
    }
    @Override
    public void periodic() {
        if(ShuffleboardConstants.UPDATE_SHOOTER){
            m_motorVelocity.setDouble(shooter.getMotorRPM());
            m_wheelVelocity.setDouble(shooter.getWheelRPM());
        }
    }

    public static double getShooterDutyCycle(){
        return m_dutyCycle.getDouble(0);
    }

    public static double getShooterVelocity(){
        return m_velocity.getDouble(0);
    }
}
