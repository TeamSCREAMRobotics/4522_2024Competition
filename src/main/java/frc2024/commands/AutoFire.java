package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.PIDConstants;
import com.team1706.SmartShooting;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShootState;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class AutoFire extends Command{

    Swerve swerve;
    Elevator elevator;
    Pivot pivot;
    Shooter shooter;
    Conveyor conveyor;
    LED led;
    DoubleSupplier[] translation;
    static final Interpolator<Double> pivotDistanceInterpolator = Interpolator.forDouble();

    PIDController rotationController;

    public AutoFire(DoubleSupplier[] translationSup, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, LED led){
        addRequirements(swerve, pivot, shooter);
        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.led = led;
        this.translation = translationSup;
        rotationController = SwerveConstants.VISION_ROTATION_CONSTANTS.toPIDController();
    }
    
    @Override
    public void execute() {
        double rotationValue = Math.abs(Vision.getTX(Limelight.SHOOTER)) < 2.0 ? 0 : rotationController.calculate(Vision.getTX(Limelight.SHOOTER), 0.0);
        Translation2d translationValue = translation == null ? new Translation2d() : new Translation2d(translation[0].getAsDouble(), translation[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
        swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(translationValue, rotationValue));

        if(Vision.getTV(Limelight.SHOOTER)){
            shooter.setTargetVelocity(calculateShotTrajectory(() -> elevator.getElevatorHeight()).velocityRPM());
            pivot.setTargetAngle(calculateShotTrajectory(() -> elevator.getElevatorHeight()).pivotAngle());
        }

        led.scaledTarget(Color.kGoldenrod, shooter.getRPM(), shooter.getTargetVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetVelocity(ShooterConstants.IDLE_VELOCITY);
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return !conveyor.hasPiece(false).getAsBoolean();
    }


    public static double getDistanceToSpeaker(){
        return Vision.getDistanceToTargetMeters(FieldConstants.SPEAKER_TAG_HEIGHT, Limelight.SHOOTER);
    }

    // From FRC 1757 Wolverines 
    // https://github.com/1757WestwoodRobotics/2024-Crescendo/blob/main/commands/shooter/alignandaim.py

    public static ShootState calculateShotTrajectory(DoubleSupplier elevatorHeight) {
        double distanceToTarget = getDistanceToSpeaker() + getPivotDistanceFromLens(elevatorHeight);
        double absoluteElevatorHeight = Units.inchesToMeters(elevatorHeight.getAsDouble()) + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR - Units.inchesToMeters(21); // 21.0 // + ElevatorConstants.HOME_HEIGHT_FROM_FLOOR;
        double shooterDistance = PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP;// - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE;

        double extraYVel = Conversions.falconRPSToMechanismMPS(ShooterConstants.TRAJECTORY_VELOCITY_EXTRA / 60.0, ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0);

        double vy = Math.sqrt(
                extraYVel * extraYVel
                + (FieldConstants.SPEAKER_OPENING_HEIGHT - (absoluteElevatorHeight - shooterDistance)) * 2 * SmartShooting.GRAVITY
        );
        
        double airtime = (vy - extraYVel) / SmartShooting.GRAVITY;
        double vx = distanceToTarget / airtime;

        double launchAngleRads = Math.atan2(vy, vx);
        Rotation2d launchAngle = Rotation2d.fromRadians(MathUtil.clamp(-launchAngleRads + PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL.getRadians(), Math.toRadians(1.0), Math.toRadians(45.0)));
        double launchVelRPM = MathUtil.clamp(Conversions.mpsToFalconRPS(Math.sqrt(vx * vx + vy * vy), ShooterConstants.WHEEL_CIRCUMFERENCE, 1.0) * 60.0, 0, ShooterConstants.SHOOTER_MAX_VELOCITY); // rpm

        return new ShootState(launchAngle, 0.0, launchVelRPM);
    } 

    public static double getPivotDistanceFromLens(DoubleSupplier elevatorHeight){
         return pivotDistanceInterpolator.interpolate(PivotConstants.AXLE_DISTANCE_FROM_LENS_HOME, PivotConstants.AXLE_DISTANCE_FROM_LENS_TOP, elevatorHeight.getAsDouble() / ElevatorConstants.MAX_HEIGHT);
    }
}
