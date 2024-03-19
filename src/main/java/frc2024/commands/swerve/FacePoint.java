// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.swerve;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import com.team1706.FieldRelativeAccel;
import com.team1706.SmartShooting;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class FacePoint extends Command {

  Swerve swerve;
  Pivot pivot;
  Shooter shooter;
  DoubleSupplier[] translationSup;
  Rotation2d targetAngle;
  PIDController targetController;
  Translation2d target;
  boolean front;
  boolean virtualCalculation;

  /** Face Point Command Without Virtual Targeting */
  public FacePoint(Swerve swerve, DoubleSupplier[] translationSup, Translation2d target, boolean intakeSide) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.pivot = null;
    this.shooter = null;
    this.translationSup = translationSup;
    this.target = target;
    this.virtualCalculation = false;
    targetController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    targetController.enableContinuousInput(-180.0, 180.0);
  }

  /** Face Point Command With Virtual Targeting */
  public FacePoint(Swerve swerve, Pivot pivot, Shooter shooter, DoubleSupplier[] translationSup, Translation2d target, boolean intakeSide) {
    addRequirements(swerve, pivot, shooter);

    this.swerve = swerve;
    this.pivot = pivot;
    this.shooter = shooter;
    this.translationSup = translationSup;
    this.target = target;
    this.virtualCalculation = true;
    targetController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    targetController.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(virtualCalculation){
      target = SmartShooting.calculateVirtualTarget(swerve, pivot, shooter, target);
    }

    Translation2d drivingTranslation = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());

    targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getPose().getTranslation(), target);
    /* Substract PI if the robot should face the point with the back of the robot */
    if(!front) targetAngle = targetAngle.minus(Rotation2d.fromRadians(Math.PI));

    swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(drivingTranslation, targetController.calculate(swerve.getHeading().getDegrees(), targetAngle.getDegrees())));
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
