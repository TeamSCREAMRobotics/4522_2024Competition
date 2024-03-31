// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.DoubleSupplier;

import javax.swing.text.StyledEditorKit.AlignmentAction;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.RectanglePoseArea;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class Feed extends Command {
  
  Swerve swerve;
  Pivot pivot;
  Elevator elevator;
  Shooter shooter;
  Conveyor conveyor;
  LED led;

  DoubleSupplier[] translationSup;

  Translation3d targetPoint;
  RectanglePoseArea illegalArea;
  int directionCoefficient;

  public Feed(DoubleSupplier[] translationSup, Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor, LED led) {
    addRequirements(swerve, pivot, elevator, shooter, led);
    setName("Feed");
    this.swerve = swerve;
    this.pivot = pivot;
    this.elevator = elevator;
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.led = led;
    this.translationSup = translationSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPoint = AllianceFlipUtil.MirroredTranslation3d(new Translation3d(2.71, 6.02, 7.0));
    illegalArea = AllianceFlipUtil.PoseArea(FieldConstants.WING_POSE_AREA);
    directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double horizontalDistance = ScreamUtil.calculateDistanceToTranslation(swerve.getEstimatedPose().getTranslation(), targetPoint.toTranslation2d());
    ShootState targetState = ShootingUtil.calculateShootState(targetPoint.getZ(), horizontalDistance, elevator.getElevatorHeight());
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getEstimatedPose().getTranslation(), targetPoint.toTranslation2d()).minus(new Rotation2d(Math.PI));
    Translation2d translation = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times((SwerveConstants.MAX_SPEED * 0.8) * directionCoefficient);

    swerve.setChassisSpeeds(swerve.snappedFieldRelativeSpeeds(translation, targetAngle));
    
    if(!illegalArea.isPoseWithinArea(swerve.getEstimatedPose())){
      shooter.setTargetVelocity(targetState.velocityRPM() / 2.93);
      pivot.setTargetAngle(targetState.pivotAngle());
      elevator.setTargetHeight(ElevatorConstants.SUBWOOFER_HEIGHT);
      led.strobe(Color.kGreen, 0.3);
    } else {
      led.strobe(Color.kRed, 0.3);
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
