// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.swerve;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import com.team1706.FieldRelativeAccel;
import com.team1706.FieldRelativeSpeed;
import com.team1706.SmartShooting;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.swerve.Swerve;

public class FacePointCommand extends Command {

  Swerve swerve;
  DoubleSupplier[] translationSup;
  Rotation2d targetAngle;
  PIDController targetController;
  Translation2d target;
  Translation2d referenceTarget;
  boolean front;
  boolean virtualCalculation;

  public FacePointCommand(Swerve swerve, DoubleSupplier[] translationSup, Translation2d target, boolean front, boolean virtualCalculation) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.translationSup = translationSup;
    this.target = target;
    this.referenceTarget = target;
    this.virtualCalculation = virtualCalculation;
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
      target = SmartShooting.calculateVirtualTarget(swerve, referenceTarget);
    }

    Translation2d drivingTranslation = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());

    targetAngle = ScreamUtil.calculateYawToPose(swerve.getPose(), new Pose2d(target, Rotation2d.fromDegrees(0)));
    /* Substract PI if the robot should face the point with the front of the robot */
    if(front) targetAngle = targetAngle.minus(Rotation2d.fromRadians(Math.PI));

    swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(drivingTranslation, targetController.calculate(swerve.getRotation().getDegrees(), targetAngle.getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static Rotation2d calculateAngleToPoint(Translation2d current, Translation2d target){
    double targetX = target.getX() - current.getX();
    double targetY = target.getY() - current.getY();
    /* adding PI faces the point with the backside of the robot */
    return Rotation2d.fromRadians(Math.atan2(targetY, targetX)).plus(Rotation2d.fromRadians(Math.PI));
  }
}
