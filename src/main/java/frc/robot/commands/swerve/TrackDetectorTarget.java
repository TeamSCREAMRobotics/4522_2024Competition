// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.Ports;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;

public class TrackDetectorTarget extends Command {
  /** Creates a new TrackVisionTarget. */
  Swerve swerve;
  PIDController xController;
  PIDController yController;
  PIDController rotController;

  Rotation2d rotTarget;//AllianceFlippable.Rotation2d(Rotation2d.fromDegrees(0.0));

  double xError;
  double yError;
  
  public TrackDetectorTarget(Swerve swerve, ScreamPIDConstants translationXConstants, ScreamPIDConstants translationYConstants, ScreamPIDConstants rotationConstants) {
    addRequirements(swerve);
    this.swerve = swerve;
    xController = translationXConstants.toPIDController();
    yController = translationYConstants.toPIDController();
    rotController = rotationConstants.toPIDController();

    xController.setTolerance(0.1);
    yController.setTolerance(0.5);
    rotController.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(Ports.LIMELIGHT_FRONT, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xValue = LimelightHelpers.getTV(Ports.LIMELIGHT_FRONT) ? -yController.calculate(LimelightHelpers.getTY(Ports.LIMELIGHT_FRONT), VisionConstants.DETECTOR_TARGET_TY) : 0;
    double yValue = LimelightHelpers.getTV(Ports.LIMELIGHT_FRONT) ? xController.calculate(LimelightHelpers.getTX(Ports.LIMELIGHT_FRONT), VisionConstants.DETECTOR_TARGET_TX) : 0;
    //double rotValue = rotController.calculate(swerve.getYaw().getDegrees(), rotTarget.getDegrees());
    swerve.setChassisSpeeds(
      swerve.robotRelativeSpeeds(new Translation2d(xValue, yValue), 0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint();// && yController.atSetpoint();// && rotController.atSetpoint();
  }
}
