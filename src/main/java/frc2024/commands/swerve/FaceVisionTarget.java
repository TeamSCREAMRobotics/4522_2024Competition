// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.IntakePipeline;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class FaceVisionTarget extends Command {
  Swerve swerve;
  PIDController rotController;
  DoubleSupplier[] translationSup;
  IntakePipeline pipeline;
  Limelight limelight;
  
  public FaceVisionTarget(Swerve swerve, DoubleSupplier[] translationSup, ScreamPIDConstants rotationConstants, Limelight limelight) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.translationSup = translationSup;
    rotController = rotationConstants.toPIDController();
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    Translation2d translationValue = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
    double rotationValue = Math.abs(Vision.getTX(limelight)) < 2.0 ? 0 : rotController.calculate(Vision.getTX(limelight), 0.0);

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
