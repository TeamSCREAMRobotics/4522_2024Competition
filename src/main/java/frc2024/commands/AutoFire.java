package frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
                .alongWith(pivot.angleCommand(() -> ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).pivotAngle()).alongWith(elevator.heightCommand(() -> ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).elevatorHeightInches())))
                .onlyWhile(
                    () -> runCondition(swerve))
/*                 .until(
                    () -> endCondition(swerve, shooter, pivot, elevator)) */
                .finallyDo(
                    () -> {
                            // conveyor.setConveyorOutput(ConveyorConstants.SHOOT_SPEED);
                        }
                )
        );
    }

    public double getDistanceToSpeaker(Swerve swerve){
        return ScreamUtil.calculateDistanceToTranslation(Vision.getBotPose2d(Limelight.SHOOTER).getTranslation(), AllianceFlippable.getTargetSpeaker().getTranslation());
    }

    public boolean runCondition(Swerve swerve){
        return getDistanceToSpeaker(swerve) <= 4.5;
    }

    public boolean endCondition(Swerve swerve, Shooter shooter, Pivot pivot, Elevator elevator){
        return shooter.getRPM() >= ShooterConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker(swerve)).velocityRPM()-100
           && pivot.getPivotAtTarget()
           && elevator.getElevatorAtTarget();
    }
}
