// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class DriveToPositionCommand extends Command {
  
  Swerve swerve;
  Translation3d targetPosition;
  PIDController driveController;
  PIDController rotationController;

  public DriveToPositionCommand(Swerve swerve, Translation3d targetPosition) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.targetPosition = targetPosition;
    this.driveController = SwerveConstants.DRIVE_TO_TARGET_CONSTANTS.toPIDController();
    this.rotationController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    rotationController.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationValue = new Translation2d(driveController.calculate(swerve.getPose().getX(), targetPosition.getX()), driveController.calculate(swerve.getPose().getY(), targetPosition.getY()));
    double rotationValue = rotationController.calculate(swerve.getRotation().getDegrees(), new Rotation2d(targetPosition.getZ()).getDegrees());

    swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(translationValue, rotationValue));
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
