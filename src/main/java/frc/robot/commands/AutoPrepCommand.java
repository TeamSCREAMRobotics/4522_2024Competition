// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class AutoPrepCommand extends Command {
  
  Pivot pivot;
  Elevator elevator;
  Shooter shooter;
  Swerve swerve;
  boolean defense;

  Rotation2d wantedPivotAngle;
  double wantedElevatorHeight;
  double distanceFromSpeaker_X;
  double distanceFromSpeaker_Y;
  double distanceFromSpeaker;

  Translation2d allianceSpeakerPosition;

  public AutoPrepCommand(Pivot pivot, Elevator elevator, Shooter shooter, Swerve swerve, boolean defense) {
    addRequirements(pivot, elevator, shooter, swerve);

    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.swerve = swerve;
    this.defense = defense;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceSpeakerPosition = AllianceFlippable.Translation2d(FieldConstants.BLUE_SPEAKER_OPENING, FieldConstants.RED_SPEAKER_OPENING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceFromSpeaker_X = Math.abs(swerve.getPose().getX() - allianceSpeakerPosition.getX());
    distanceFromSpeaker_Y = Math.abs(swerve.getPose().getY() - allianceSpeakerPosition.getY());
    distanceFromSpeaker = new Translation2d(distanceFromSpeaker_X, distanceFromSpeaker_Y).getNorm();
  
    shooter.setTargetVelocity(ShooterConstants.SHOOTER_TARGET_VELOCITY);
    if(!defense){
      pivot.setTargetAngle(Rotation2d.fromDegrees(PivotConstants.pivotAngleMap_Localization.get(distanceFromSpeaker)));
      elevator.setTargetHeight(ElevatorConstants.elevatorHeightMap_Localization.get(distanceFromSpeaker));
    }
    else{
      pivot.setTargetAngle(Rotation2d.fromDegrees(PivotConstants.pivotAngleMap_Defense.get(distanceFromSpeaker)));
      elevator.setTargetHeight(ElevatorConstants.elevatorHeightMap_Defense.get(distanceFromSpeaker));
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
