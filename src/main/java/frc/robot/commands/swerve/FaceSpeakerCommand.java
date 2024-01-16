// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class FaceSpeakerCommand extends Command {

  Swerve swerve;
  Alliance allianceColor;
  DoubleSupplier[] drivingTranslationSupplier;
  Rotation2d targetAngle;
  Translation3d allianceSpeakerPose;
  PIDController targetController;
  Translation2d targetPose;

  public FaceSpeakerCommand(Swerve swerve, Alliance allianceColor, DoubleSupplier[] drivingTranslation, Translation2d targetPose) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.allianceColor = allianceColor;
    this.drivingTranslationSupplier = drivingTranslation;
    this.targetPose = targetPose;
    targetController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    targetController.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d drivingTranslation = new Translation2d(drivingTranslationSupplier[0].getAsDouble(), drivingTranslationSupplier[1].getAsDouble()).times(AllianceFlippable.DirectionCoefficient()).times(SwerveConstants.MAX_SPEED);

    double targetX = allianceSpeakerPose.getX() - swerve.getPose().getX();
    double targetY = allianceSpeakerPose.getY() - swerve.getPose().getY();
    targetAngle = Rotation2d.fromRadians(Math.atan2(targetY, targetX));

    swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(drivingTranslation, targetController.calculate(swerve.getRotation().getDegrees(), targetAngle.getDegrees())));
    System.out.println("Target Angle: " + targetAngle.getDegrees() + "\n" + "Target Output: " + targetController.calculate(targetAngle.getDegrees()));
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
