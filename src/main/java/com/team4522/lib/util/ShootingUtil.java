package com.team4522.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.util.Units;
import frc2024.Constants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.VisionConstants;

public class ShootingUtil {

    static final Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();
    static final Interpolator<Double> curveDistanceInterpolator = Interpolator.forDouble();

    public static double calculateTOF(double elevatorHeightInches, double targetHeightMeters){
      return 2 * (targetHeightMeters - calculateAbsolutePivotPosition(elevatorHeightInches).getY()) / Constants.GRAVITY;
    }

    public static Translation2d calculateAbsolutePivotPosition(double elevatorHeightInches){
      return new Translation2d(-robotCenterToPivot(elevatorHeightInches), (Units.inchesToMeters(elevatorHeightInches) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR) - PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP);
    }

    public static ShootState calculateShootState(double targetHeightMeters, double horizontalDistance, double elevatorHeightInches){
      Rotation2d baseAngle = new Translation2d(horizontalDistance, targetHeightMeters).minus(calculateAbsolutePivotPosition(elevatorHeightInches)).getAngle();
      /* Rotation2d adjustedAngle = Rotation2d.fromDegrees(MathUtil.clamp(-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees(), 1, 45)).plus(calculatePivotAngleAdjustment()); */
      Rotation2d adjustedAngle = Rotation2d.fromDegrees(MathUtil.clamp((-baseAngle.getDegrees() + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getDegrees()) - (horizontalDistance / 6), 1, 45));

      //double tof = calculateTOF(elevatorHeightInches, targetHeightMeters);
      double velocityRPM = horizontalDistance * 1000;

      return new ShootState(adjustedAngle, VisionConstants.ELEVATOR_HEIGHT_MAP.get(horizontalDistance), velocityRPM);
    }

    /* private Rotation2d calculateTargetRobotAngle(){
      double robotVelocity = Math.sqrt(Math.pow(robotSpeed.vxMetersPerSecond, 2) + Math.pow(robotSpeed.vyMetersPerSecond, 2));

      Rotation2d movingAngleAdjustment = Rotation2d.fromRadians(Math.atan2(robotSpeed.vyMetersPerSecond, robotVelocity));

      return ScreamUtil.calculateAngleToPoint(swerve.getPose().getTranslation(), targetSpeaker.plus(calculateCurveOffset())).plus(movingAngleAdjustment).minus(Rotation2d.fromRadians(Math.PI));
    } */

    /* private Rotation2d calculatePivotAngleAdjustment(){
      double scaleFactor = 0.5;
      return Rotation2d.fromRadians(scaleFactor * Math.tan(robotSpeed.vxMetersPerSecond / calculateHorizontalDistance()));
    } */

    public static double robotCenterToPivot(double elevatorHeightInches){
      return pivotDistanceInterpolator.interpolate(PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_HOME, PivotConstants.AXLE_DISTANCE_FROM_ROBOT_CENTER_TOP, elevatorHeightInches / ElevatorConstants.MAX_HEIGHT);
    }

    public static Translation2d calculateCurveOffset(double horizontalDistance){
      return new Translation2d(0, curveDistanceInterpolator.interpolate(0.3, 1.0, horizontalDistance / 6.0) * AllianceFlipUtil.getDirectionCoefficient());
    }
}
