// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.auto;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants;
import frc2024.Constants.ConveyorConstants;
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
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class AutoPoseShootingContinuous extends Command {
  Swerve swerve;
  Pivot pivot;
  Elevator elevator;
  Shooter shooter;
  Conveyor conveyor;

  Translation2d targetSpeaker;
  double directionCoefficient;

  final Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();

  public AutoPoseShootingContinuous(Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(pivot, elevator, shooter);
    setName("AutoPoseShootingContinuous");
    this.swerve = swerve;
    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.conveyor = conveyor;
  }

  @Override
  public void initialize() {
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
    targetSpeaker = AllianceFlipUtil.getTargetSpeaker().getTranslation().plus(new Translation2d(Units.inchesToMeters(12.0) * directionCoefficient, Units.inchesToMeters(8) * directionCoefficient));
  }

  @Override
  public void execute() {
    double horizontalDistance = ScreamUtil.calculateDistanceToTranslation(swerve.getEstimatedPose().getTranslation(), targetSpeaker);
    ShootState targetState = ShootingUtil.calculateShootState(FieldConstants.SPEAKER_OPENING_HEIGHT, horizontalDistance, elevator.getElevatorHeight());
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getEstimatedPose().getTranslation(), targetSpeaker).minus(new Rotation2d(Math.PI));
    Rotation2d adjustedPivotAngle = 
      Rotation2d.fromDegrees(
        MathUtil.clamp(
          targetState.pivotAngle().getDegrees(), 
          1, 
          28)
      );

    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(targetAngle));

    //swerve.resetPose_Apriltag();
    //swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(new Translation2d(), targetAngle));
    elevator.setTargetHeight(targetState.elevatorHeightInches());
    shooter.setTargetVelocity(MathUtil.clamp(targetState.velocityRPM(), 3000.0, 5000.0));
    pivot.setTargetAngle(adjustedPivotAngle);
  }

  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
