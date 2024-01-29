// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.IntakePipeline;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.swerve.Swerve;

public class FaceVisionTargetCommand extends Command {
  Swerve swerve;
  PIDController rotController;
  Supplier<Translation2d> translationSup;
  IntakePipeline pipeline;
  
  public FaceVisionTargetCommand(Swerve swerve, Supplier<Translation2d> translationSup, ScreamPIDConstants rotationConstants, IntakePipeline pipeline) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.translationSup = translationSup;
    this.pipeline = pipeline;
    rotController = rotationConstants.toPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Vision.setPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    Translation2d translationValue = translationSup.get().times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
    double rotationValue = Math.abs(Vision.getTX(Limelight.INTAKE)) < 2.0 ? 0 : rotController.calculate(Vision.getTX(Limelight.INTAKE), 0.0);

    swerve.setChassisSpeeds(swerve.robotRelativeSpeeds(translationValue, rotationValue));
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
