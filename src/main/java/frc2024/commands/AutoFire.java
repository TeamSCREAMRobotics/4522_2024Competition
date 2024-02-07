package frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class AutoFire extends SequentialCommandGroup{

    public AutoFire(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, BooleanSupplier defense){
        addCommands(
            shooter.velocityCommand(ShooterConstants.SHOOTER_TARGET_VELOCITY)
                .alongWith(
                    defense.getAsBoolean() 
                        ? pivot.angleCommand(Rotation2d.fromDegrees(PivotConstants.ANGLE_MAP_DEFENDED.get(getDistanceFromSpeaker(swerve))))  
                        : pivot.angleCommand(Rotation2d.fromDegrees(PivotConstants.ANGLE_MAP_UNDEFENDED.get(getDistanceFromSpeaker(swerve))))
                        .alongWith(
                            defense.getAsBoolean()
                                ? elevator.positionCommand(ElevatorConstants.MAX_HEIGHT)
                                : elevator.positionCommand(ElevatorConstants.HEIGHT_MAP.get(getDistanceFromSpeaker(swerve)))))
                .onlyWhile(() -> runCondition(swerve))
                .until(() -> endCondition(swerve, shooter, pivot, elevator))
                .finallyDo(() -> conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED).withTimeout(0.2).andThen(new GoHome(elevator, pivot)))
        );
    }

    public double getDistanceFromSpeaker(Swerve swerve){
        return Vision.getDistanceFromSpeaker(Limelight.SHOOTER);
    }

    public boolean runCondition(Swerve swerve){
        return getDistanceFromSpeaker(swerve) <= ShooterConstants.AUTO_SHOOT_DISTANCE_THRESHOLD;
    }

    public boolean endCondition(Swerve swerve, Shooter shooter, Pivot pivot, Elevator elevator){
        return shooter.getWheelRPM() > ShooterConstants.minumumShooterOutput.get(getDistanceFromSpeaker(swerve))
           && pivot.getPivotAtTarget()
           && elevator.getElevatorAtTarget();
    }
}
