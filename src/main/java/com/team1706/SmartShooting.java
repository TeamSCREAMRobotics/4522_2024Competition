package com.team1706;

import java.util.function.DoubleSupplier;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc2024.Constants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class SmartShooting {

  public static final double GRAVITY = 9.80665;
  public static final double xOffset = AllianceFlipUtil.Number(0.0, -0.0);
  public static final double yOffset = AllianceFlipUtil.Number(0.25, -0.25);
  public static final double forwardOffset = AllianceFlipUtil.Number(12.0, -12.0);
  public static final double shotTimeOffset = 0.0;
  public static final double targetHeight = FieldConstants.SPEAKER_OPENING_HEIGHT;
  public static final double kAccelCompFactor = 0.08; //0.065

  //1706 mock-up, shoot while moving from Rapid React 2022 Season
  public static Translation2d calculateVirtualTarget(Swerve swerve, Pivot pivot, Shooter shooter, Translation2d target){
    Translation2d currentPose = swerve.getEstimatedPose().getTranslation();

    /* Gets the field relative speed variables */
    ChassisSpeeds m_fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(), swerve.getEstimatedHeading());
    ChassisSpeeds m_lastFieldRelVel = m_fieldRelVel;
    FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, Constants.LOOP_TIME_SEC);

    /* Finds the robot distance to the physical target */
    double distance = (Math.sqrt(Math.pow(getDistanceToTarget_SHOOTER(swerve, target, targetHeight), 2) + Math.pow(targetHeight, 2)));
    /* Gets the time of flight for the note based on the physical target */
    double shotTime = calculateShotTimeToGoal(pivot, shooter, distance);
    Translation2d movingGoalLocation = new Translation2d();
      for(int i=0; i<5; i++){
        /* Calculates and sets the virtual target location */
        double virtualGoalX = target.getX() - shotTime * (m_fieldRelVel.vxMetersPerSecond + m_fieldRelAccel.ax * kAccelCompFactor);
        double virtualGoalY = target.getY() - shotTime * (m_fieldRelVel.vyMetersPerSecond + m_fieldRelAccel.ay * kAccelCompFactor);
        Translation2d virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
        /* Sets the horizontal distance to the goal */
        double horizontalVirtualDistance = ScreamUtil.calculateDistanceToTranslation(virtualGoalLocation, currentPose);
        double totalVirtualDistance = (Math.sqrt(Math.pow(horizontalVirtualDistance, 2) + Math.pow(targetHeight, 2)));
        /* Gets the new time of flight for the note based on the virtual target */
        double newShotTime = calculateShotTimeToGoal(pivot, shooter, totalVirtualDistance);
        
        /* Checks the error between the previous shot times */
        if(Math.abs(newShotTime-shotTime) <= 0.01){
            i = 4;
        }
        if(i == 4){
            /* Sets the moving goal location based on the virtual goal */
            movingGoalLocation = new Translation2d(virtualGoalLocation.getX()+xOffset, virtualGoalLocation.getY()+yOffset);
        }
        else shotTime = newShotTime;
      }

    System.out.println("Virtual location: " + movingGoalLocation + "\n" + "Shot Time: " + shotTime + "\n" + "Pose: " + currentPose);
    return movingGoalLocation;
  }

  /* Calculates the time of flight based on a distance */ //TODO don't believe it includes elevator height
  public static double calculateShotTimeToGoal(Pivot pivot, Shooter shooter, double distanceToTarget){
    Rotation2d pivotAngle = pivot.getPivotAngle();
    double shooterRPMs = shooter.getRPM();
    double wheelRadius = Units.inchesToMeters(4.0);
    
    /* Conversion factor for RPM to radians per second */
    double rpmToRadiansPerSecond = 2 * Math.PI / 60.0;

    /* Calculate angular velocity in radians per second */
    double angularVelocity = shooterRPMs * rpmToRadiansPerSecond;

    /* Calculate initial velocity components */
    if(angularVelocity == 0) angularVelocity = 1;
    double initialVelocityX = wheelRadius * angularVelocity * Math.cos(pivotAngle.getRadians());
    double initialVelocityY = wheelRadius * angularVelocity * Math.sin(pivotAngle.getRadians());

    /* Calculate horizontal distance */
    double horizontalDistance = Math.sqrt(Math.pow(distanceToTarget, 2) - Math.pow(targetHeight, 2));
    
    /* Calculate time to reach horizontal distance */
    double timeToHorizontalDistance = horizontalDistance / initialVelocityX;
    
    /* Calculate vertical height reached during time to reach horizontal distance */
    double verticalHeightReached = initialVelocityY * timeToHorizontalDistance - 0.5 * GRAVITY * Math.pow(timeToHorizontalDistance, 2);
    
    /* Calculate total vertical height (initial height + height reached during horizontal motion) */
    double totalVerticalHeight = PivotConstants.AXLE_HEIGHT_HOME + verticalHeightReached;

    /* Calculate time of flight using the total vertical height */
    double timeToGoal = Math.sqrt(2 * totalVerticalHeight / GRAVITY) + timeToHorizontalDistance;

    //TODO jank fix to unknown error with null times
    if(!(timeToGoal > 0)){
      return 1.0;
    }

    /* Returns the calculated time minus the set offset */
    return timeToGoal-shotTimeOffset;
  }

  /* Calculates the distance to a specified target based on the shooter side limelight */
  public static double getDistanceToTarget_SHOOTER(Swerve swerve, Translation2d target, double targetHeight){
    /* Sets the stationary pivot height */
    double pivotHeight = PivotConstants.AXLE_HEIGHT_HOME;
    /* Gets the current pose of the robot on the field */
    Translation2d currentPosition = swerve.getEstimatedPose().getTranslation();

    /* Calculates the height difference for calulcations */
    double height_diff = targetHeight - pivotHeight;
    /* Gets the horizontal distance based on the current pose and target location */
    double horizontalDistance = currentPosition.getDistance(target);
    /* Calculates the total distance to the target */
    double totalDistance = Math.hypot(horizontalDistance, height_diff);

    return totalDistance;
  }

  /* Calculates the pivot angle, elevator height, and shooterRPMs */
  public static ShootState calculateShotTrajectory(DoubleSupplier elevatorHeight, double distanceToTarget) {
    /* Sets the targetHeight */
    double targetHeight = FieldConstants.SPEAKER_OPENING_HEIGHT;
    /* Calculates the height difference */
    double heightDiff = targetHeight - PivotConstants.AXLE_HEIGHT_HOME;
    /* Calculates the horizontal distance to the target */
    double horizontalDistance = Math.sqrt(Math.pow(distanceToTarget, 2) - Math.pow(heightDiff, 2));
    
    /* Rotation2d baseAngle = new Translation2d(horizontalDistance, FieldConstants.SPEAKER_OPENING_HEIGHT).minus(ShootingUtil.calculateAbsolutePivotPosition(elevatorHeight.getAsDouble())).getAngle();
    Rotation2d launchAngle = Rotation2d.fromDegrees(MathUtil.clamp((-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees()) - (horizontalDistance / 6), 1, 45));

    double launchVelRPM = MathUtil.clamp(horizontalDistance * 1000, ShooterConstants.SUBWOOFER_VELOCITY, ShooterConstants.SHOOTER_MAX_VELOCITY);
    launchVelRPM += ShooterConstants.shooterOffset.get(horizontalDistance); */

    return ShootingUtil.oldCalculateShootState(targetHeight, horizontalDistance, elevatorHeight.getAsDouble());
  }

  private static PIDController targetController;
  /* Gets the rotation value to a point */
  public static double getRotationToPoint(Swerve swerve, Pivot pivot, Shooter shooter, Translation2d target, boolean virtualCalculation, boolean front, PIDController PIDController){
    targetController = PIDController;
    targetController.enableContinuousInput(-180.0, 180.0);

    /* Offsets the target location forward by a set amount */
    target = new Translation2d(target.getX() + Units.inchesToMeters(forwardOffset), target.getY());

    if(virtualCalculation){
      /* Calculates the virtual target location */
      target = SmartShooting.calculateVirtualTarget(swerve, pivot, shooter, target);
    }

    /* Calculates the rotation angle based on the current pose and the target location */
    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getEstimatedPose().getTranslation(), target);

    /* Substract PI if the robot should face the point with the back of the robot */
    if(!front) targetAngle = targetAngle.minus(Rotation2d.fromRadians(Math.PI));

    /* Calculates the value for the chassis speeds in terms of a PID Controller */
    double targetValue = targetController.calculate(swerve.getEstimatedHeading().getDegrees(), targetAngle.getDegrees());

    return targetValue;
  }
}
