// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.swerve.Swerve;

public class DriveToPose extends Command {
  
  Swerve swerve;
  Pose2d targetPose;
  PIDController driveController;

  public DriveToPose(Swerve swerve, Pose2d targetPose) {
    addRequirements(swerve);
    setName("DriveToPose");

    this.swerve = swerve;
    this.targetPose = targetPose;
    this.driveController = SwerveConstants.DRIVE_TO_TARGET_CONSTANTS.toPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationValue = new Translation2d(driveController.calculate(swerve.getEstimatedPose().getX(), targetPose.getX()), driveController.calculate(swerve.getEstimatedPose().getY(), targetPose.getY()));

    swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(translationValue, targetPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
