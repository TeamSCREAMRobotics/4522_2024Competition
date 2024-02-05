// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import org.photonvision.PhotonUtils;

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

public class AutoPrepCommand extends Command {
  
  Pivot pivot;
  Elevator elevator;
  Shooter shooter;
  Swerve swerve;
  boolean defense;

  double distanceFromSpeaker;

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceFromSpeaker = ScreamUtil.calculateDistanceToPose(swerve.getPose(), AllianceFlippable.getTargetSpeaker());
  
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
