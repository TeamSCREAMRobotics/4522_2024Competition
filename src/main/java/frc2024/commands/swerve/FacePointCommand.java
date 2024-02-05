// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.swerve;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import com.team1706.FieldRelativeAccel;
import com.team1706.FieldRelativeSpeed;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants;
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
      target = calculateVirtualTarget(swerve, referenceTarget);
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

  //1706 mock-up, shoot while moving from Rapid React 2022 Season
  public static Translation2d calculateVirtualTarget(Swerve swerve, Translation2d target){
    /* Creates the field relative speed variables */
    FieldRelativeSpeed m_FieldRelVel = new FieldRelativeSpeed();
    FieldRelativeSpeed m_LastFieldRelVel = new FieldRelativeSpeed();
    FieldRelativeAccel m_FieldRelAccel = new FieldRelativeAccel();

    /* Sets the field relative speed variables */
    m_FieldRelVel = new FieldRelativeSpeed(swerve.getRobotRelativeSpeeds(), swerve.getYaw());
    m_FieldRelAccel = new FieldRelativeAccel(m_FieldRelVel, m_LastFieldRelVel, Constants.LOOP_TIME_SEC);
    m_LastFieldRelVel = m_FieldRelVel;

    FieldRelativeSpeed robotVel = m_FieldRelVel;
    FieldRelativeAccel robotAccel = m_FieldRelAccel;

    /* Finds the robot distance to the target */
    Translation2d robotToGoal = target.minus(swerve.getPose().getTranslation());
    double dist = robotToGoal.getDistance(new Translation2d());
    
    /* Gets the time it takes for the note to leave the shooter TODO necessary to have it a changing value? (refrence constants) */
    double shotTime = 0.3;//ShooterConstants.timeToGoalMap.get(dist);
    Translation2d movingGoalLocation = new Translation2d();
    for(int i=0; i<5; i++){
      /* Calculates the  distance and time for the virtual target */
        double virtualGoalX = target.getX() - shotTime * (robotVel.vx + robotAccel.ax * 0.01 /* ShooterConstants.kAccelCompFactor */);
        //TODO shooter constant comp factor? It is necessary but why? How do you get the value?
        double virtualGoalY = target.getY() - shotTime * (robotVel.vy + robotAccel.ay * 0.01 /* ShooterConstants.kAccelCompFactor */);
        //TODO shooter constant comp factor? It is necessary but why? How do you get the value?
        Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
        Translation2d toGoalDistance = testGoalLocation.minus(swerve.getPose().getTranslation());
        double newShotTime = 0.3;//ShooterConstants.timeToGoalMap.get(toGoalDistance.getDistance(new Translation2d()));
        
        //Used to smooth the angle adjustment of the robot
        if(Math.abs(newShotTime-shotTime) <= 0.01){
            i = 4;
        }
        if(i == 4){
            movingGoalLocation = testGoalLocation;
        }
        else shotTime = newShotTime;
    }

    return movingGoalLocation;
  }
}
