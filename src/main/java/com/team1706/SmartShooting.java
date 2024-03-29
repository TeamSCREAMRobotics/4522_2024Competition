package com.team1706;

import java.util.function.DoubleSupplier;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;

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
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class SmartShooting {

  public static final double GRAVITY = 9.80665;
  public static final double xOffset = AllianceFlipUtil.Number(0.0, -0.0);
  public static final double yOffset = AllianceFlipUtil.Number(0.25, -0.25);
  public static final double shotTimeOffset = 0.0;
  public static final double targetHeight = FieldConstants.SPEAKER_OPENING_HEIGHT; /* FieldConstants.SPEAKER_TAG_HEIGHT; */

  //1706 mock-up, shoot while moving from Rapid React 2022 Season
  public static Translation2d calculateVirtualTarget(Swerve swerve, Pivot pivot, Shooter shooter, Translation2d target){
    Translation2d currentPose = /* Vision.getBotPose2d(Limelight.SHOOT_SIDE).getTranslation(); */ swerve.getPose().getTranslation();

    /* Sets the field relative speed variables */
    ChassisSpeeds m_fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(), swerve.getHeading());
    ChassisSpeeds m_lastFieldRelVel = m_fieldRelVel;
    FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, Constants.LOOP_TIME_SEC);

    /* Finds the robot distance to the physical target */
    double distance = (Math.sqrt(Math.pow(getDistanceToTarget_SHOOTER(swerve, target, targetHeight), 2) + Math.pow(targetHeight, 2)));
    double shotTime = calculateShotTimeToGoal(pivot, shooter, distance);
    Translation2d movingGoalLocation = new Translation2d();
    for(int i=0; i<5; i++){
      /* Calculates the  distance and time for the virtual target */
        double virtualGoalX = target.getX() - shotTime * (m_fieldRelVel.vxMetersPerSecond + m_fieldRelAccel.ax * 0.065 /* ShooterConstants.kAccelCompFactor */);
        double virtualGoalY = target.getY() - shotTime * (m_fieldRelVel.vyMetersPerSecond + m_fieldRelAccel.ay * 0.065 /* ShooterConstants.kAccelCompFactor */);
        Translation2d virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
        double toGoalDistance = ScreamUtil.calculateDistanceToTranslation(virtualGoalLocation, currentPose);
        double virtualDistance = (Math.sqrt(Math.pow(toGoalDistance, 2) + Math.pow(targetHeight, 2)));
        double newShotTime = calculateShotTimeToGoal(pivot, shooter, virtualDistance);
        
        //Used to smooth the angle adjustment of the robot
        if(Math.abs(newShotTime-shotTime) <= 0.01){
            i = 4;
        }
        if(i == 4){
            movingGoalLocation = new Translation2d(virtualGoalLocation.getX()+xOffset, virtualGoalLocation.getY()+yOffset);
        }
        else shotTime = newShotTime;
    }

    System.out.println("Virtual location: " + movingGoalLocation + "\n" + "Shot Time: " + shotTime + "\n" + "Pose: " + currentPose);
    return movingGoalLocation;
  }

  public static double calculateShotTimeToGoal(Pivot pivot, Shooter shooter, double distanceToTarget){
    Rotation2d pivotAngle = pivot.getPivotAngle();
    double shooterRPMs = shooter.getRPM();
    double wheelRadius = Units.inchesToMeters(4.0);
    
    // Conversion constants
    double rpmToRadiansPerSecond = 2 * Math.PI / 60.0; // Conversion factor for RPM to radians per second

    // Calculate angular velocity in radians per second
    double angularVelocity = shooterRPMs * rpmToRadiansPerSecond;

    // Calculate initial velocity components
    if(angularVelocity == 0) angularVelocity = 1;
    double initialVelocityX = wheelRadius * angularVelocity * Math.cos(pivotAngle.getRadians());
    double initialVelocityY = wheelRadius * angularVelocity * Math.sin(pivotAngle.getRadians());

    // Calculate horizontal distance
    double horizontalDistance = Math.sqrt(Math.pow(distanceToTarget, 2) - Math.pow(targetHeight, 2));
    
    // Calculate time to reach horizontal distance
    double timeToHorizontalDistance = horizontalDistance / initialVelocityX;
    
    // Calculate vertical height reached during time to reach horizontal distance
    double verticalHeightReached = initialVelocityY * timeToHorizontalDistance - 0.5 * GRAVITY * Math.pow(timeToHorizontalDistance, 2);
    
    // Calculate total vertical height (initial height + height reached during horizontal motion)
    double totalVerticalHeight = PivotConstants.AXLE_HEIGHT_HOME + verticalHeightReached;

    // Calculate time of flight using the total vertical height
    double timeToGoal = Math.sqrt(2 * totalVerticalHeight / GRAVITY) + timeToHorizontalDistance;

    if(!(timeToGoal > 0)){
      return 1.0; //If timeToGoal is not a number then it returns a value of 1.0
    }

    return timeToGoal-shotTimeOffset;
  }

  //** Calculates the distance to a specified target based on the shooter side limelight */
  public static double getDistanceToTarget_SHOOTER(Swerve swerve, Translation2d target, double targetHeight){
    double pivotHeight = PivotConstants.AXLE_HEIGHT_HOME;
    Translation2d currentPosition = swerve.getPose().getTranslation(); /* Vision.getBotPose2d(Limelight.SHOOT_SIDE).getTranslation(); */

    double height_diff = targetHeight - pivotHeight;
    double horizontalDistance = currentPosition.getDistance(target);
    double totalDistance = Math.hypot(horizontalDistance, height_diff);

    return totalDistance;
  }

  public static ShootState calculateShotTrajectory(DoubleSupplier elevatorHeight, double distanceToTarget) {
    /* double absoluteElevatorHeight = Units.inchesToMeters(elevatorHeight.getAsDouble()) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR - Units.inchesToMeters(12.0);
    double shooterDistance = PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE;

    double extraYVel = Conversions.falconRPSToMechanismMPS(ShooterConstants.TRAJECTORY_VELOCITY_EXTRA / 60.0, ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0);

    double vy = Math.sqrt(
      extraYVel * extraYVel
        + (FieldConstants.SPEAKER_OPENING_HEIGHT - (absoluteElevatorHeight - shooterDistance)) * 2 * GRAVITY
    );
      
    double airtime = (vy - extraYVel) / GRAVITY;
    double vx = distanceToTarget / airtime;
    double launchAngleRads = Math.atan2(vy, vx);
    Rotation2d launchAngle = Rotation2d.fromRadians(launchAngleRads).unaryMinus().plus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL);
    double launchVelRPM = Conversions.mpsToFalconRPS(Math.sqrt(vx * vx + vy * vy), ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0) * 60.0; // rpm */

    double heightDiff = FieldConstants.SPEAKER_OPENING_HEIGHT - PivotConstants.AXLE_HEIGHT_HOME;
    double horizontalDistance = Math.sqrt(Math.pow(distanceToTarget, 2) - Math.pow(heightDiff, 2));
    
    Rotation2d baseAngle = new Translation2d(horizontalDistance, FieldConstants.SPEAKER_OPENING_HEIGHT).minus(calculateAbsolutePivotPosition(elevatorHeight.getAsDouble())).getAngle();
    Rotation2d launchAngle = Rotation2d.fromDegrees(MathUtil.clamp((-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees()) - (horizontalDistance / 6), 1, 45));

    double launchVelRPM = MathUtil.clamp(horizontalDistance * 1000, ShooterConstants.SUBWOOFER_VELOCITY, ShooterConstants.SHOOTER_MAX_VELOCITY);
    launchVelRPM += ShooterConstants.shooterOffset.get(horizontalDistance);

    return new ShootState(launchAngle, 0.0, launchVelRPM);
  }
  
  private static Translation2d calculateAbsolutePivotPosition(double elevatorHeight){
    return new Translation2d(-robotCenterToPivot(elevatorHeight), (Units.inchesToMeters(elevatorHeight) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR) - PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP);
  }

  final static Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();
  private static double robotCenterToPivot(double elevatorHeight){
    return pivotDistanceInterpolator.interpolate(PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_HOME, PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_TOP, elevatorHeight / ElevatorConstants.MAX_HEIGHT);
  }

  private static PIDController targetController;

  public static double getRotationToPoint(Swerve swerve, Pivot pivot, Shooter shooter, Translation2d target, boolean virtualCalculation, boolean front, PIDController PIDController){
    targetController = PIDController;
    targetController.enableContinuousInput(-180.0, 180.0);

    target = new Translation2d(target.getX() + Units.inchesToMeters(15.0), target.getY()); //Offsets the target 8 inches forward

    if(virtualCalculation){
      target = SmartShooting.calculateVirtualTarget(swerve, pivot, shooter, target);
    }

    Rotation2d targetAngle = ScreamUtil.calculateAngleToPoint(swerve.getPose().getTranslation() /* Vision.getBotPose2d(Limelight.SHOOT_SIDE).getTranslation() */, target);
    /* Substract PI if the robot should face the point with the back of the robot */
    if(!front) targetAngle = targetAngle.minus(Rotation2d.fromRadians(Math.PI));

    double targetValue = targetController.calculate(swerve.getHeading().getDegrees(), targetAngle.getDegrees());

    return targetValue;
  }
}
