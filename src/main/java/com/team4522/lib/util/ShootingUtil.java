package com.team4522.lib.util;

import com.team4522.lib.math.Conversions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc2024.Constants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.VisionConstants;

public class ShootingUtil {

    public static double calculateTOF(double elevatorHeightInches, double targetHeightMeters){
      return 2 * (targetHeightMeters - calculateAbsolutePivotPosition(elevatorHeightInches).getY()) / Constants.GRAVITY;
    }

    public static Translation2d calculateAbsolutePivotPosition(double elevatorHeightInches){
      return new Translation2d(robotCenterToPivot(elevatorHeightInches), (Units.inchesToMeters(elevatorHeightInches) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR) - PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP);
    }

    public static ShootState calculateShootState(double targetHeightMeters, double horizontalDistance, double elevatorHeightInches){
      Rotation2d baseAngle = new Translation2d(horizontalDistance, targetHeightMeters - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE).minus(calculateAbsolutePivotPosition(elevatorHeightInches)).getAngle();
      /* Rotation2d adjustedAngle = Rotation2d.fromDegrees(MathUtil.clamp(-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees(), 1, 45)).plus(calculatePivotAngleAdjustment()); */
      Rotation2d adjustedAngle = Rotation2d.fromDegrees((-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees()));

      //double tof = calculateTOF(elevatorHeightInches, targetHeightMeters);
      double velocityRPM = horizontalDistance * (1500.0 - 175.0); // - 300.0

      return new ShootState(adjustedAngle, VisionConstants.ELEVATOR_HEIGHT_MAP.get(Units.metersToFeet(horizontalDistance)), velocityRPM);
    }

    public static ShootState oldCalculateShootState(double targetHeightMeters, double horizontalDistance, double elevatorHeightInches){
      Translation2d absolutePivotPosition = calculateAbsolutePivotPosition(elevatorHeightInches);
      double distanceToTarget = horizontalDistance + absolutePivotPosition.getX();
      double extraYVel = Conversions.falconRPSToMechanismMPS(ShooterConstants.TRAJECTORY_VELOCITY_EXTRA / 60.0, ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0);
      double vy = Math.sqrt(
              extraYVel * extraYVel
              + ((targetHeightMeters - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE) - (absolutePivotPosition.getY())) * 2 * Constants.GRAVITY
      );
      
      double airtime = (vy - extraYVel) / Constants.GRAVITY;
      double vx = distanceToTarget / airtime;
      double launchAngleRads = Math.atan2(vy, vx);
      Rotation2d launchAngle = Rotation2d.fromRadians(-launchAngleRads + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getRadians());
      double launchVelRPM = Conversions.mpsToFalconRPS(Math.sqrt(vx * vx + vy * vy), ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0) * 60.0; // rpm
      return new ShootState(launchAngle, VisionConstants.SHOOT_STATE_MAP.get(Units.metersToFeet(horizontalDistance)).elevatorHeightInches(), launchVelRPM); 
    }

    /* private Rotation2d calculateTargetRobotAngle(){
      double robotVelocity = Math.sqrt(Math.pow(robotSpeed.vxMetersPerSecond, 2) + Math.pow(robotSpeed.vyMetersPerSecond, 2));

      Rotation2d movingAngleAdjustment = Rotation2d.fromRadians(Math.atan2(robotSpeed.vyMetersPerSecond, robotVelocity));

      return ScreamUtil.calculateAngleToPoint(swerve.getPose().getTranslation(), targetSpeaker.plus(calculateCurveOffset())).plus(movingAngleAdjustment).minus(Rotation2d.fromRadians(Math.PI));
    } */

    public static  Rotation2d calculatePivotAngleAdjustment(ChassisSpeeds robotSpeed, double horizontalDistance, double scalingFactor){
      return Rotation2d.fromDegrees(scalingFactor * robotSpeed.vxMetersPerSecond);
    }

    public static double robotCenterToPivot(double elevatorHeightInches){
      return MathUtil.interpolate(PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_HOME, PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_TOP, elevatorHeightInches / ElevatorConstants.MAX_HEIGHT);
    }

    public static Translation2d calculateCurveOffset(double horizontalDistance){
      return new Translation2d(0, MathUtil.interpolate(0.3, 1.0, horizontalDistance / 6.0) * AllianceFlipUtil.getDirectionCoefficient());
    }
    
    /* Determines what field pose to aim. Corrects issues with shot accuracy from the sides of the goals */
    public static Translation2d determineGoalLocation(Pose2d pose){
      Translation2d speaker = AllianceFlipUtil.getTargetSpeaker().getTranslation();
      int directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
      if(pose.getY() < 6.0 && pose.getY() > 5.0){
        return speaker.plus(FieldConstants.SPEAKER_GOAL_OFFSET_CENTER.times(directionCoefficient));
      } else if(AllianceFlipUtil.Boolean(pose.getY() > 6.0, pose.getY() < 5.0)) {
        return speaker.plus(FieldConstants.SPEAKER_GOAL_OFFSET_LEFT.times(directionCoefficient));
      } else if(AllianceFlipUtil.Boolean(pose.getY() < 5.0, pose.getY() > 6.0)) {
        return speaker.plus(FieldConstants.SPEAKER_GOAL_OFFSET_RIGHT.times(directionCoefficient));
      } else {
        return speaker;
      }
    }
}
