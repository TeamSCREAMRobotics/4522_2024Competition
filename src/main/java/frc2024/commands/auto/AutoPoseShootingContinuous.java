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
import frc2024.RobotContainer;
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

  Translation2d targetPoint;
  double directionCoefficient;

  Rotation2d overrideAngle = null;

  boolean rotTargetOverride;

  public AutoPoseShootingContinuous(boolean rotTargetOverride, Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(shooter);
    setName("AutoPoseShootingContinuous");
    this.swerve = swerve;
    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.rotTargetOverride = rotTargetOverride;
  }

  @Override
  public void initialize() {
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
    targetPoint = AllianceFlipUtil.getTargetSpeaker().getTranslation().plus(FieldConstants.SPEAKER_GOAL_OFFSET_RIGHT.times(directionCoefficient));
  }

  @Override
  public void execute() {
    double horizontalDistance = ScreamUtil.calculateDistanceToTranslation(swerve.getEstimatedPose().getTranslation(), targetPoint);
    targetPoint = ShootingUtil.determineGoalLocation(swerve.getEstimatedPose());
    ShootState targetState = ShootingUtil.calculateShootState(FieldConstants.SPEAKER_OPENING_HEIGHT, horizontalDistance, elevator.getElevatorHeight());
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getEstimatedPose().getTranslation(), targetPoint).minus(new Rotation2d(Math.PI));
    Rotation2d adjustedPivotAngle = 
      Rotation2d.fromDegrees(
        MathUtil.clamp(
          targetState.pivotAngle().getDegrees() - (horizontalDistance / 3.0), 
          1, 
          28)
      );

    if(rotTargetOverride){
      PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(targetAngle));
    }

    if(!RobotContainer.isRunningPath){
      swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(new Translation2d(), targetAngle, Rotation2d.fromDegrees(1.5)));
    }
    //swerve.resetPose_Apriltag();
    elevator.setTargetHeight(targetState.elevatorHeightInches());
    shooter.setTargetVelocity(MathUtil.clamp(targetState.velocityRPM(), 3500.0, 5000.0));
    pivot.setTargetAngle(getTargetAngle(adjustedPivotAngle));
  }

  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Rotation2d getTargetAngle(Rotation2d measurement){
    if(overrideAngle != null){
      return overrideAngle;
    }
    return measurement;
  }

  public void overrideTargetAngle(Rotation2d angle){
    overrideAngle = angle;
  }
}
