// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;
import com.team6328.GeomUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

public class PoseAutoFire extends Command {
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
  final Interpolator<Double> curveDistanceInterpolator = Interpolator.forDouble();

  public PoseAutoFire(DoubleSupplier[] translationSup, Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor, LED led) {
    addRequirements(swerve, pivot, elevator, shooter, conveyor, led);
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
    targetSpeaker = AllianceFlipUtil.getTargetSpeaker().getTranslation();
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
  }

  @Override
  public void execute() {
    robotSpeed = swerve.getFieldRelativeSpeeds();

    ShootState targetState = calculateShootState();
    Rotation2d targetAngle = calculateTargetRobotAngle();
    Translation2d translation = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times((SwerveConstants.MAX_SPEED * 0.5) * directionCoefficient);

    swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(translation, targetAngle));
    elevator.setTargetHeight(targetState.elevatorHeightInches());
    shooter.setTargetVelocity(targetState.velocityRPM());
    pivot.setTargetAngle(targetState.pivotAngle());
    led.scaledTarget(Color.kOrange, shooter.getRPM(), shooter.getTargetVelocity());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateHorizontalDistance(){
    return swerve.getPose().getTranslation().getDistance(targetSpeaker);
  }

  private double calculateTOF(){
    return 2 * (FieldConstants.SPEAKER_OPENING_HEIGHT - calculateAbsolutePivotPosition().getY()) / Constants.GRAVITY;
  }

  private Translation2d calculateAbsolutePivotPosition(){
    return new Translation2d(-robotCenterToPivot(), (Units.inchesToMeters(elevator.getElevatorHeight()) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR) - PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP);
  }

  private ShootState calculateShootState(){
    Rotation2d baseAngle = new Translation2d(calculateHorizontalDistance(), FieldConstants.SPEAKER_OPENING_HEIGHT).minus(calculateAbsolutePivotPosition()).getAngle();
    double adjustedAngle = MathUtil.clamp((-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees()) + calculatePivotAngleAdjustment().getDegrees(), PivotConstants.SUBWOOFER_ANGLE.getDegrees(), 45);

    double tof = calculateTOF();
    double velocityRPM = MathUtil.clamp((Conversions.mpsToFalconRPS(calculateHorizontalDistance() / tof, ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0) * 60.0) + ShooterConstants.ARBITRARY_VELOCITY_EXTRA, ShooterConstants.SUBWOOFER_VELOCITY, ShooterConstants.SHOOTER_MAX_VELOCITY);

    return new ShootState(Rotation2d.fromDegrees(adjustedAngle), VisionConstants.ELEVATOR_HEIGHT_MAP.get(calculateHorizontalDistance()), velocityRPM);
  }

  private Rotation2d calculateTargetRobotAngle(){
    double robotVelocity = Math.sqrt(Math.pow(robotSpeed.vxMetersPerSecond, 2) + Math.pow(robotSpeed.vyMetersPerSecond, 2));

    Rotation2d movingAngleAdjustment = Rotation2d.fromRadians(Math.atan2(robotSpeed.vyMetersPerSecond, robotVelocity));

    return ScreamUtil.calculateAngleToPoint(swerve.getPose().getTranslation(), targetSpeaker.plus(calculateCurveOffset())).plus(movingAngleAdjustment);
  }

  private Rotation2d calculatePivotAngleAdjustment(){
    double scaleFactor = 0.5;
    return Rotation2d.fromRadians(scaleFactor * Math.tan(robotSpeed.vxMetersPerSecond / calculateHorizontalDistance()));
  }

  public double robotCenterToPivot(){
    return pivotDistanceInterpolator.interpolate(PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_HOME, PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_TOP, elevator.getElevatorHeight() / ElevatorConstants.MAX_HEIGHT);
  }

  private Translation2d calculateCurveOffset(){
    return new Translation2d(0, curveDistanceInterpolator.interpolate(0.3, 1.0, calculateHorizontalDistance() / 6.0) * directionCoefficient);
  }
}
