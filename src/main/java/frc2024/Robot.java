// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2024.controlboard.Controlboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Timer timeSinceDisabled = new Timer();

  public Robot() {}

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    timeSinceDisabled.reset();
    timeSinceDisabled.start();
  }

  @Override
  public void disabledPeriodic() {
    if(((int) timeSinceDisabled.get()) == 5){
      RobotContainer.getSwerve().setNeutralModes(NeutralModeValue.Coast, NeutralModeValue.Coast);
    }
  }

  @Override
  public void disabledExit() {
    RobotContainer.getSwerve().setNeutralModes(NeutralModeValue.Brake, NeutralModeValue.Brake);
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.stopAll();
  }

  @Override
  public void autonomousInit() {
    RobotContainer.setAlliance(DriverStation.getAlliance().get());
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void driverStationConnected() {
    RobotContainer.setAlliance(DriverStation.getAlliance().get());
  }
}
