// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024;

import java.util.ResourceBundle.Control;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.OrchestraUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.VisionConstants;
import frc2024.commands.AutoFire;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Timer timeSinceDisabled = new Timer();

  public Robot() {}

  @Override
  public void robotInit() {
    /* System.out.println("[Init] Starting AdvantageKit");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.MODE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        Logger.start();
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        Logger.start();
        break;
      case SIM, DEV:
        break;
      default:
        break;
    } */
  // Start logging! No more data receivers, replay sources, or metadata values may be added.

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println("(" + RobotContainer.getPivot().getPivotAngle().getDegrees() + ", " + RobotContainer.getElevator().getElevatorHeight() + ", " + RobotContainer.getShooter().getRPM() + ")");
  }

  @Override
  public void disabledInit() {
    timeSinceDisabled.reset();
    timeSinceDisabled.start();
    RobotContainer.stopAll();
  }

  @Override
  public void disabledPeriodic() {
    if(((int) timeSinceDisabled.get()) == 5){
      //RobotContainer.getSwerve().setNeutralModes(NeutralModeValue.Coast, NeutralModeValue.Coast);
      //RobotContainer.getPivot().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void disabledExit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.setAllNeutralModes(NeutralModeValue.Brake);
    RobotContainer.stopAll();
    timeSinceDisabled.stop();
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
  public void autonomousExit() {
    RobotContainer.stopAll();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
  }

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
