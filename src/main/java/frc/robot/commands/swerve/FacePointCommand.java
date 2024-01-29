// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class FacePointCommand extends Command {

  Swerve swerve;
  Supplier<Translation2d> translationSup;
  Rotation2d targetAngle;
  PIDController targetController;
  Translation2d target;

  public FacePointCommand(Swerve swerve, Supplier<Translation2d> translationSup, Translation2d target) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.translationSup = translationSup;
    this.target = target;
    targetController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    targetController.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d drivingTranslation = translationSup.get().times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
    targetAngle = calculateAngleToPoint(swerve.getPose().getTranslation(), target);

    swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(drivingTranslation, targetController.calculate(swerve.getRotation().getDegrees(), targetAngle.getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static Rotation2d calculateAngleToPoint(Translation2d current, Translation2d target){
    double targetX = target.getX() - current.getX();
    double targetY = target.getY() - current.getY();
    return Rotation2d.fromRadians(Math.atan2(targetY, targetX)).plus(Rotation2d.fromRadians(Math.PI));
  }
}
