// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.AllianceFlippable;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.LimelightHelpers.LimelightResults;
import frc.lib.util.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.Constants.Ports;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;

public class TrackDetectorTarget extends Command {
  /** Creates a new TrackVisionTarget. */
  Swerve swerve;
  PIDController xController;
  PIDController yController;
  PIDController rotController;

  double xDirection = AllianceFlippable.Number(1, -1);
  double yDirection = AllianceFlippable.Number(-1, 1);
  Rotation2d rotTarget = AllianceFlippable.Rotation2d(Rotation2d.fromDegrees(0.0));

  LimelightResults results = LimelightHelpers.getLatestResults(Ports.LIMELIGHT_FRONT);
  LimelightTarget_Detector[] detector = results.targetingResults.targets_Detector;
  LimelightTarget_Detector target = detector[Math.round((detector.length - 1)/2)];
  
  public TrackDetectorTarget(Swerve swerve, ScreamPIDConstants translationConstants, ScreamPIDConstants rotationConstants) {
    this.swerve = swerve;
    xController = translationConstants.toPIDController();
    yController = translationConstants.clone().toPIDController();
    rotController = rotationConstants.toPIDController();

    xController.setTolerance(0.1);
    yController.setTolerance(0.25);
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
    swerve.setChassisSpeeds(
      new ChassisSpeeds(
        yController.calculate(target.ty, VisionConstants.DETECTOR_TARGET_TY) * yDirection, 
        xController.calculate(target.tx, VisionConstants.DETECTOR_TARGET_TX * xDirection), 
        rotController.calculate(swerve.getYaw().getDegrees(), rotTarget.getDegrees()))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }
}
