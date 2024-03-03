package frc2024.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple4;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.RobotContainer;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.VisionConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class AutoFire extends SequentialCommandGroup{

    public static final double GRAVITY = 9.80665; 
    public static final Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();

    /* public AutoFire(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, BooleanSupplier defense){
        addCommands(
            shooter.velocityCommand(() -> defense.getAsBoolean() ?
                VisionConstants.SHOOT_STATE_MAP_DEFENDED.get(getDistanceToSpeaker().getAsDouble()).velocityRPM() :
                VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).velocityRPM())
                    .alongWith(
                        pivot.angleCommand(() -> defense.getAsBoolean() ?
                            VisionConstants.SHOOT_STATE_MAP_DEFENDED.get(getDistanceToSpeaker().getAsDouble()).pivotAngle() :
                            VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).pivotAngle())
                                .alongWith(elevator.heightCommand(() -> defense.getAsBoolean() ?
                                    VisionConstants.SHOOT_STATE_MAP_DEFENDED.get(getDistanceToSpeaker().getAsDouble()).elevatorHeightInches() :
                                    VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).elevatorHeightInches())))
                    .onlyWhile(() -> getDistanceToSpeaker().isPresent())
        );
    } */

    public AutoFire(Shooter shooter, Elevator elevator, Pivot pivot, BooleanSupplier defense){
        addCommands(
            shooter.velocityCommand(() -> calculateShotTrajectory(() -> elevator.getElevatorHeight()).velocityRPM())
/*                 .alongWith(elevator.heightCommand(
                    () -> defense.getAsBoolean() ? ElevatorConstants.MAX_HEIGHT : ElevatorConstants.MIN_HEIGHT)) */
                .alongWith(pivot.angleCommand(() -> calculateShotTrajectory(() -> elevator.getElevatorHeight()).pivotAngle()))
            .onlyWhile(() -> Vision.getTV(Limelight.SHOOTER))
        );
    }

    public static double getDistanceToSpeaker(){
        return Vision.getDistanceToTargetMeters(FieldConstants.SPEAKER_TAG_HEIGHT, Limelight.SHOOTER);
    }

    // From FRC 1757 Wolverines 
    // https://github.com/1757WestwoodRobotics/2024-Crescendo/blob/main/commands/shooter/alignandaim.py

    public static ShootState calculateShotTrajectory(DoubleSupplier elevatorHeight) {
        double distanceToTarget = getDistanceToSpeaker() + getPivotDistanceFromLens(elevatorHeight);
        double absoluteElevatorHeight = Units.inchesToMeters(elevatorHeight.getAsDouble()) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR - Units.inchesToMeters(12.0);// + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR;
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
        double launchVelRPM = Conversions.mpsToFalconRPS(Math.sqrt(vx * vx + vy * vy), ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0) * 60.0; // rpm

        return new ShootState(launchAngle, 0.0, launchVelRPM);
    }

    public static double getPivotDistanceFromLens(DoubleSupplier elevatorHeight){
         return pivotDistanceInterpolator.interpolate(PivotConstants.AXLE_DISTANCE_FROM_LENS_HOME, PivotConstants.AXLE_DISTANCE_FROM_LENS_TOP, elevatorHeight.getAsDouble() / ElevatorConstants.MAX_HEIGHT);
    }
}
