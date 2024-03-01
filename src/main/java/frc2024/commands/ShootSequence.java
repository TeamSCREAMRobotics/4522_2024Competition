package frc2024.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.SwerveConstants;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class ShootSequence extends SequentialCommandGroup{

    Debouncer test = new Debouncer(0.5, DebounceType.kBoth);

        public ShootSequence(Swerve swerve, Pivot pivot, Shooter shooter, Elevator elevator, Conveyor conveyor){
            addCommands(
                new ParallelCommandGroup(
                    new AutoFire(shooter, elevator, pivot, () -> false),
                    new FaceVisionTarget(swerve, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                .until(() -> test.calculate(shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean()))
                .andThen(conveyor.dutyCycleCommand(ConveyorConstants.SHOOT_SPEED).until(() -> !conveyor.hasPiece().getAsBoolean()))
                .andThen(conveyor.stopCommand().alongWith(shooter.idleCommand()))
            );
        }

        public ShootSequence(SuperstructureState state, double velocity, Pivot pivot, Shooter shooter, Elevator elevator, Conveyor conveyor){
            addCommands(
                new ParallelCommandGroup(
                    new SuperstructureToPosition(state, elevator, pivot),
                    shooter.velocityCommand(velocity)
                )
                .until(()-> shooter.getShooterAtTarget().getAsBoolean() && elevator.getElevatorAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean())
                .andThen(conveyor.dutyCycleCommand(ConveyorConstants.SHOOT_SPEED).withTimeout(0.5))//.until(() -> !conveyor.hasPiece().getAsBoolean()))
                .andThen(conveyor.stopCommand().alongWith(shooter.idleCommand())
                .alongWith(new SuperstructureToPosition(SuperstructureState.HOME, elevator, pivot))
                    .until(() -> elevator.getElevatorAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean()))
            );
        }
    }