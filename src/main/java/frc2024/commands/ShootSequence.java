package frc2024.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class ShootSequence extends SequentialCommandGroup{

        public ShootSequence(Swerve swerve, Pivot pivot, Shooter shooter, Elevator elevator, Conveyor conveyor){
            addCommands(
                new ParallelCommandGroup(
                    new AutoFire(shooter, elevator, pivot, () -> false),
                    new FaceVisionTarget(swerve, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                .until(() -> shooter.getShooterAtTarget() && pivot.getPivotAtTarget())
                .andThen(conveyor.dutyCycleCommand(ConveyorConstants.SHOOT_SPEED).until(() -> !conveyor.hasPiece().getAsBoolean()))
                .andThen(conveyor.stopCommand().alongWith(shooter.stopCommand()))
            );
        }
    }