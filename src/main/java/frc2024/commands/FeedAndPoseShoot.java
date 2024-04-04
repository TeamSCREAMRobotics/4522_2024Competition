// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.RectanglePoseArea;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class FeedAndPoseShoot extends Command {
  DoubleSupplier[] translation;
  BooleanSupplier defense;
  Swerve swerve;
  Elevator elevator;
  Pivot pivot;
  Shooter shooter;
  Conveyor conveyor;
  Intake intake;
  LED led;

  Translation3d targetPoint;
  double directionCoefficient;

  RectanglePoseArea illegalArea;

  boolean shouldFeed;

  public FeedAndPoseShoot(DoubleSupplier[] translation, BooleanSupplier defense, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake, LED led) {
    this.translation = translation;
    this.defense = defense;
    this.swerve = swerve;
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.intake = intake;
    this.led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
    targetPoint = shouldFeed ? AllianceFlipUtil.MirroredTranslation3d(new Translation3d(2, 5.6, 7.0)) : new Translation3d(AllianceFlipUtil.getTargetSpeaker().getTranslation().getX() + Units.inchesToMeters(12.0 * directionCoefficient), AllianceFlipUtil.getTargetSpeaker().getTranslation().getY() + Units.inchesToMeters(8.0 * directionCoefficient), FieldConstants.SPEAKER_OPENING_HEIGHT);
    illegalArea = AllianceFlipUtil.PoseArea(FieldConstants.WING_POSE_AREA);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizontalDistance = ScreamUtil.calculateDistanceToTranslation(swerve.getEstimatedPose().getTranslation(), targetPoint.toTranslation2d());
    boolean shouldFeed = horizontalDistance >= 7.3;
    ShootState targetState = ShootingUtil.calculateShootState(targetPoint.getZ(), horizontalDistance, elevator.getElevatorHeight());
    Rotation2d adjustedPivotAngle = 
      Rotation2d.fromDegrees(
        MathUtil.clamp(
          targetState.pivotAngle().getDegrees(), 
          elevator.getElevatorHeight() > 1.5 ? PivotConstants.SUBWOOFER_ANGLE.getDegrees() : 1, 
          28));
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getEstimatedPose().getTranslation(), targetPoint.toTranslation2d()).minus(new Rotation2d(Math.PI));
    Translation2d driveTranslation = new Translation2d(translation[0].getAsDouble(), translation[1].getAsDouble());

    if(shouldFeed){
      if(!illegalArea.isPoseWithinArea(swerve.getEstimatedPose())){
      shooter.setTargetVelocity(MathUtil.clamp(targetState.velocityRPM() / 2.8, 0.0, 3000.0));
      pivot.setTargetAngle(adjustedPivotAngle);
      elevator.setTargetHeight(ElevatorConstants.SUBWOOFER_HEIGHT);
      led.strobe(Color.kGreen, 0.3);
      if(shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean()){
        conveyor.setConveyorOutput(ConveyorConstants.SHOOT_OUTPUT);
        intake.setIntakeOutput(IntakeConstants.INTAKE_OUTPUT);
      }
      } else {
        led.strobe(Color.kRed, 0.3);
      }
    } else {
      swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(driveTranslation, targetAngle));
      elevator.setTargetHeight(defense.getAsBoolean() && swerve.snappedToAngle(45.0) ? ElevatorConstants.MAX_HEIGHT : targetState.elevatorHeightInches());
      shooter.setTargetVelocity(MathUtil.clamp(targetState.velocityRPM() + ShooterConstants.ARBITRARY_VELOCITY_EXTRA, 3250.0, ShooterConstants.SHOOTER_MAX_VELOCITY));
      pivot.setTargetAngle(adjustedPivotAngle);

      if((shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean() && shooter.getRPM() > ShooterConstants.TARGET_THRESHOLD && swerve.snappedToAngle(8.5))){
        led.scaledTarget(Color.kGreen, shooter.getRPM(), shooter.getTargetVelocity());
      } else {
        led.scaledTarget(Color.kOrange, shooter.getRPM(), shooter.getTargetVelocity());
      }
    }
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
