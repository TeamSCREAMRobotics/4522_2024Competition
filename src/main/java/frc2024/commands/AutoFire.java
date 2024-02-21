package frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.ElevatorPivotPosition;
import frc2024.Constants.PivotConstants;
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

    public AutoFire(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, BooleanSupplier defense){
        addCommands(
            shooter.velocityCommand(ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).velocityRPM())
                .alongWith(
                    new SuperstructureToPosition(
                        defense.getAsBoolean()
                        ? ElevatorConstants.MAX_HEIGHT
                        : ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).elevatorHeightInches(),
                        defense.getAsBoolean()
                        ? Rotation2d.fromDegrees(PivotConstants.ANGLE_MAP_DEFENDED.get(getDistanceToSpeaker(swerve)).doubleValue())
                        : ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).pivotAngle(),
                        elevator,
                        pivot
                    )
                .onlyWhile(
                    () -> runCondition(swerve))
                .until(
                    () -> endCondition(swerve, shooter, pivot, elevator))
                .finallyDo(
                    () -> conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED)
                        .withTimeout(0.2)
                        .andThen(
                            new SuperstructureToPosition(ElevatorPivotPosition.HOME, elevator, pivot)
                                .alongWith(shooter.stopCommand().alongWith(conveyor.stopCommand())))))
        );
    }

    public double getDistanceToSpeaker(Swerve swerve){
        return Vision.getFusedDistanceToSpeaker(Limelight.SHOOTER, swerve.getPose());
    }

    public boolean runCondition(Swerve swerve){
        return getDistanceToSpeaker(swerve) <= ShooterConstants.AUTO_SHOOT_DISTANCE_THRESHOLD;
    }

    public boolean endCondition(Swerve swerve, Shooter shooter, Pivot pivot, Elevator elevator){
        return shooter.getRPM() >= ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).velocityRPM()
           && pivot.getPivotAtTarget()
           && elevator.getElevatorAtTarget()
           && ScreamUtil.valueWithinThreshold(Vision.getTX(Limelight.SHOOTER), VisionConstants.AUTO_FIRE_X_THRESHOLD);
    }
}
