package frc2024.commands;

import java.util.Optional;
import java.util.OptionalDouble;
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
import frc2024.Constants.FieldConstants;
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
            shooter.velocityCommand(VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).velocityRPM())
                .alongWith(
                    pivot.angleCommand(() -> VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).pivotAngle())
                        .alongWith(elevator.heightCommand(() -> VisionConstants.SHOOT_STATE_MAP.get(getDistanceToSpeaker().getAsDouble()).elevatorHeightInches())))
                .onlyWhile(() -> getDistanceToSpeaker().isPresent())
        );
    }

    public OptionalDouble getDistanceToSpeaker(){
        return Vision.getDistanceToTarget(FieldConstants.SPEAKER_TAG_HEIGHT, Limelight.SHOOTER);
    }
}
