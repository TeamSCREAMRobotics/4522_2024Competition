// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.swerve.Swerve;

public class AutoAlignCommand extends Command {

  Swerve swerve;
  Rotation2d targetAngle;
  double targetY;
  Limelight limelight;

  PIDController xController;
  PIDController yController;
  PIDController rotationController;

  public AutoAlignCommand(Swerve swerve, Rotation2d targetAngle, double targetY, Limelight limelight) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.targetAngle = targetAngle;
    this.targetY = targetY;
    this.limelight = limelight;

    xController = SwerveConstants.VISION_TRANSLATION_X_CONSTANTS.toPIDController();
    yController = SwerveConstants.VISION_TRANSLATION_Y_CONSTANTS.toPIDController();
    rotationController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    rotationController.enableContinuousInput(-180.0, 180.0);
    xController.setTolerance(0.5);
    yController.setTolerance(0.5);
    rotationController.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Vision.setPipeline(limelight, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xValue = Vision.getTV(limelight) ? -yController.calculate(Vision.getTY(limelight), targetY) : 0;
    double yValue = Vision.getTV(limelight) ? xController.calculate(Vision.getTX(limelight), 0.0) : 0;
    double rotValue = rotationController.calculate(swerve.getYaw().getDegrees(), targetAngle.getDegrees());

    swerve.setChassisSpeeds(swerve.robotRelativeSpeeds(new Translation2d(xValue, yValue), rotValue));
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
