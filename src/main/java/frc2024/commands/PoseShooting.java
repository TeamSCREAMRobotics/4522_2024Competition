// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootingUtil;
import com.team6328.GeomUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
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
  Intake intake;
  LED led;

  DoubleSupplier[] translationSup;

  Translation2d targetSpeaker;
  double directionCoefficient;

  BooleanSupplier isDefended;
  
  final Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();

  public PoseShooting(DoubleSupplier[] translationSup, BooleanSupplier isDefended, Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor, Intake intake, LED led) {
    addRequirements(swerve, pivot, elevator, shooter, led);
    setName("PoseShooting");
    this.swerve = swerve;
    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.intake = intake;
    this.led = led;
    this.translationSup = translationSup;
    this.isDefended = isDefended;
  }

  @Override
  public void initialize() {
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
    targetSpeaker = AllianceFlipUtil.getTargetSpeaker().getTranslation().plus(FieldConstants.SPEAKER_GOAL_OFFSET.times(directionCoefficient));
  }

  @Override
  public void execute() {
    double horizontalDistance = ScreamUtil.calculateDistanceToTranslation(swerve.getEstimatedPose().getTranslation(), targetSpeaker);
    ShootState targetState = ShootingUtil.calculateShootState(FieldConstants.SPEAKER_OPENING_HEIGHT, horizontalDistance, elevator.getElevatorHeight());
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getEstimatedPose().getTranslation(), targetSpeaker).minus(new Rotation2d(Math.PI));
    Translation2d translation = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times((SwerveConstants.MAX_SPEED * 0.5) * directionCoefficient);
    Rotation2d adjustedPivotAngle = 
      Rotation2d.fromDegrees(
        MathUtil.clamp(
          targetState.pivotAngle().getDegrees(), 
          elevator.getElevatorHeight() > 1.5 ? PivotConstants.SUBWOOFER_ANGLE.getDegrees() : 1, 
          isDefended.getAsBoolean() ? 44 : 28)
      );

    swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(translation, targetAngle, Rotation2d.fromDegrees(1.5)));
    elevator.setTargetHeight(isDefended.getAsBoolean() && swerve.atAngleThreshold(targetAngle, Rotation2d.fromDegrees(45.0)) ? ElevatorConstants.MAX_HEIGHT : targetState.elevatorHeightInches());
    shooter.setTargetVelocity(MathUtil.clamp(targetState.velocityRPM() + ShooterConstants.ARBITRARY_VELOCITY_EXTRA, 3250.0, ShooterConstants.SHOOTER_MAX_VELOCITY));
    pivot.setTargetAngle(adjustedPivotAngle);

    if(shooter.getShooterAtTarget().getAsBoolean()
        && pivot.getPivotAtTarget().getAsBoolean() 
        && shooter.getRPM() > ShooterConstants.TARGET_THRESHOLD
        && swerve.snapAtSetpoint()){
      led.scaledTarget(Color.kGreen, shooter.getRPM(), shooter.getTargetVelocity());
      conveyor.setConveyorOutput(ConveyorConstants.SHOOT_OUTPUT);
      intake.setIntakeOutput(IntakeConstants.INTAKE_OUTPUT);
    } else {
      led.scaledTarget(Color.kOrange, shooter.getRPM(), shooter.getTargetVelocity());
    }

    System.out.println("Horizontal Distance: " + horizontalDistance);
    Logger.recordOutput("Commands/PoseShooting/Distance", horizontalDistance);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
