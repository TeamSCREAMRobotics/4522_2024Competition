// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class AutoAlignCommand extends Command {

  Swerve swerve;
  Rotation2d targetAngle;
  double targetY;
  String limelightPipeline;

  PIDController xController;
  PIDController yController;
  PIDController rotController;

  public AutoAlignCommand(Swerve swerve, Rotation2d targetAngle, double targetY, String limelightPipeline) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.targetAngle = targetAngle;
    this.targetY = targetY;
    this.limelightPipeline = limelightPipeline;

    xController = SwerveConstants.VISION_TRANSLATION_X_CONSTANTS.toPIDController();
    xController.setTolerance(0.5);
    yController = SwerveConstants.VISION_TRANSLATION_Y_CONSTANTS.toPIDController();
    yController.setTolerance(0.5);
    rotController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    rotController.enableContinuousInput(-180.0, 180.0);
    rotController.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(limelightPipeline, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xValue = LimelightHelpers.getTV(limelightPipeline) ? -yController.calculate(LimelightHelpers.getTY(limelightPipeline), targetY) : 0;
    double yValue = LimelightHelpers.getTV(limelightPipeline) ? xController.calculate(LimelightHelpers.getTX(limelightPipeline), 0.0) : 0;
    double rotValue = rotController.calculate(swerve.getYaw().getDegrees(), targetAngle.getDegrees());

    swerve.setChassisSpeeds(
      swerve.robotRelativeSpeeds(new Translation2d(xValue, yValue), rotValue)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
