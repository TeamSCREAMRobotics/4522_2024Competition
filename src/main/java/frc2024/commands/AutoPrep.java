// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import org.photonvision.PhotonUtils;

import com.team1706.SmartShooting;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class AutoPrep extends Command {
  
  Pivot pivot;
  Elevator elevator;
  Shooter shooter;
  Swerve swerve;
  boolean defense;

  Rotation2d wantedPivotAngle;
  double wantedElevatorHeight;
  double distanceFromSpeaker_Physical;
  double distanceFromSpeaker_Virtual;

  public AutoPrep(Pivot pivot, Elevator elevator, Shooter shooter, Swerve swerve, boolean defense) {
    addRequirements(pivot, elevator, shooter, swerve);

    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.swerve = swerve;
    this.defense = defense;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceFromSpeaker_Physical = ScreamUtil.calculateDistanceToTranslation(swerve.getPose().getTranslation(), AllianceFlippable.getTargetSpeaker().getTranslation());
    distanceFromSpeaker_Virtual = ScreamUtil.calculateDistanceToTranslation(swerve.getPose().getTranslation(), SmartShooting.calculateVirtualTarget(swerve, AllianceFlippable.getTargetSpeaker().getTranslation()));

    double shooterTarget_RPM = ShooterConstants.MINIMUM_VELOCITY_MAP.get(distanceFromSpeaker_Virtual);
    if(shooterTarget_RPM < ShooterConstants.SHOOTER_TARGET_VELOCITY) shooterTarget_RPM = ShooterConstants.SHOOTER_TARGET_VELOCITY;

    shooter.setTargetVelocity(shooterTarget_RPM);
    if(!defense){
      pivot.setTargetAngle(Rotation2d.fromDegrees(PivotConstants.ANGLE_MAP_UNDEFENDED.get(distanceFromSpeaker_Physical)), true);
      elevator.setTargetHeight(ElevatorConstants.HEIGHT_MAP.get(distanceFromSpeaker_Physical));
    }
    else{
      pivot.setTargetAngle(Rotation2d.fromDegrees(PivotConstants.ANGLE_MAP_DEFENDED.get(distanceFromSpeaker_Physical)), true);
      elevator.setTargetHeight(ElevatorConstants.HEIGHT_MAP_DEFENDED.get(distanceFromSpeaker_Physical));
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
