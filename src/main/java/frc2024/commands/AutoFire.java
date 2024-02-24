package frc2024.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import org.opencv.core.Mat.Tuple4;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.ElevatorPivotPosition;
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

    public static final double GRAVITY = 9.802; 

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

    public AutoFire(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, BooleanSupplier defense){
        addCommands(
            shooter.velocityCommand(() -> calculateTimeVelocityAngle().velocityRPM())
                .alongWith(elevator.heightCommand(() -> calculateTimeVelocityAngle().elevatorHeightInches()))
                .alongWith(pivot.angleCommand(() -> calculateTimeVelocityAngle().pivotAngle()))
        );
    }

    public static OptionalDouble getDistanceToSpeaker(){
        return Vision.getDistanceToTargetMeters(FieldConstants.SPEAKER_TAG_HEIGHT, Limelight.SHOOTER);
    }

    public static ShootState calculateTimeVelocityAngle() {
        double distanceToTarget = getDistanceToSpeaker().getAsDouble();

        double extraYVel = Conversions.falconRPSToMechanismMPS(ShooterConstants.TRAJECTORY_VELOCITY_EXTRA / 60.0, ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0);

        double vy = Math.sqrt(
                extraYVel * extraYVel
                + (FieldConstants.SPEAKER_OPENING_HEIGHT - PivotConstants.PIVOT_HEIGHT_HOME) * 2 * GRAVITY
        );
        double airtime = (vy - extraYVel) / GRAVITY;
        double vx = distanceToTarget / airtime;

        double launchAngleRads = Math.atan2(vy, vx); // radians
        Rotation2d launchAngle = Rotation2d.fromRadians(launchAngleRads).unaryMinus().plus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL);
        double launchVel = Conversions.mpsToFalconRPS(Math.sqrt(vx * vx + vy * vy), ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0) * 60.0; // rpm

        return new ShootState(launchAngle, 0.0, VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).velocityRPM());
    }
    
    /* public Rotation2d calculateAngle(double distance){
        double x = distance + (9.0/12.0);
        double y = (78.75/12.0)-(16.5/12.0);
        return Rotation2d.fromRadians(Math.atan2(y, x));
    }

    public OptionalDouble getDistanceToSpeaker(){
        return Vision.getDistanceToTarget(FieldConstants.SPEAKER_TAG_HEIGHT, Limelight.SHOOTER);
    } */
}
