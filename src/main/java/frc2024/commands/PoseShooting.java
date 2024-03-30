// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootingUtil;
import com.team6328.GeomUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class PoseShooting extends Command {
  Swerve swerve;
  Pivot pivot;
  Elevator elevator;
  Shooter shooter;
  Conveyor conveyor;
  LED led;

  DoubleSupplier[] translationSup;

  Translation2d targetSpeaker;
  double directionCoefficient;
  ChassisSpeeds robotSpeed;

  final Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();

  public PoseShooting(DoubleSupplier[] translationSup, Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor, LED led) {
    addRequirements(swerve, pivot, elevator, shooter, led);
    this.swerve = swerve;
    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.led = led;
    this.translationSup = translationSup;
  }

  @Override
  public void initialize() {
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
    targetSpeaker = AllianceFlipUtil.getTargetSpeaker().getTranslation().plus(new Translation2d(Units.inchesToMeters(12.0) * directionCoefficient, 0));
  }

  @Override
  public void execute() {
    robotSpeed = swerve.getFieldRelativeSpeeds();

    double horizontalDistance = ScreamUtil.calculateDistanceToTranslation(swerve.getPose().getTranslation(), targetSpeaker);
    ShootState targetState = ShootingUtil.calculateShootState(FieldConstants.SPEAKER_OPENING_HEIGHT, horizontalDistance, elevator.getElevatorHeight());
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getPose().getTranslation(), targetSpeaker).minus(new Rotation2d(Math.PI));
    Translation2d translation = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times((SwerveConstants.MAX_SPEED * 0.5) * directionCoefficient);

    swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(translation, targetAngle));
    elevator.setTargetHeight(0.0 /* targetState.elevatorHeightInches() */);
    shooter.setTargetVelocity(MathUtil.clamp(targetState.velocityRPM(), 3000.0, ShooterConstants.SHOOTER_MAX_VELOCITY));
    pivot.setTargetAngle(targetState.pivotAngle());
    led.scaledTarget(Color.kOrange, shooter.getRPM(), shooter.getTargetVelocity());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
