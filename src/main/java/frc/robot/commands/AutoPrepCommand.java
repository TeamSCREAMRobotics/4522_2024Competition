// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.swerve.Swerve;

public class AutoPrepCommand extends Command {
  
  Pivot pivot;
  Elevator elevator;
  Swerve swerve;
  Alliance allianceColor;

  Rotation2d wantedPivotAngle;
  double wantedElevatorHeight;
  double distanceFromSpeaker_X;
  double distanceFromSpeaker_Y;
  double distanceFromSpeaker;

  Translation2d allianceSpeakerPose;

  public AutoPrepCommand(Pivot pivot, Elevator elevator, Swerve swerve, Alliance allianceColor) {
    addRequirements(pivot, elevator, swerve);

    this.pivot = pivot;
    this.elevator = elevator;
    this.swerve = swerve;
    this.allianceColor = allianceColor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(allianceColor == Alliance.Blue) allianceSpeakerPose = FieldConstants.BLUE_SPEAKER_OPENING;
    if(allianceColor == Alliance.Red) allianceSpeakerPose = FieldConstants.RED_SPEAKER_OPENING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceFromSpeaker_X = Math.abs(swerve.getPose().getX() - allianceSpeakerPose.getX());
    distanceFromSpeaker_Y = Math.abs(swerve.getPose().getY() - allianceSpeakerPose.getY());
    distanceFromSpeaker = new Translation2d(distanceFromSpeaker_X, distanceFromSpeaker_Y).getNorm();
  
    pivot.pivotToTargetAngle(Rotation2d.fromDegrees(PivotConstants.pivotTreeMap.get(distanceFromSpeaker)));
    elevator.toTargetHeight(ElevatorConstants.elevatorTreeMap.get(distanceFromSpeaker));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.pivotToTargetAngle(PivotConstants.pivotHome_Angle);
    elevator.toTargetHeight(ElevatorConstants.elevatorHome_Position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
