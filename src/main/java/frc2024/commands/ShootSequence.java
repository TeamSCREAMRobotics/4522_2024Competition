package frc2024.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                new InstantCommand(() -> {
                    conveyor.stop();
                }),
                new AutoFire(swerve, shooter, elevator, pivot, conveyor, () -> false)
                        .alongWith(new FaceVisionTarget(swerve, new DoubleSupplier[] {() -> 0, () -> 0}, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                            .withTimeout(2.0)
                        .alongWith(new WaitCommand(1.0)
                            .andThen(conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED).withTimeout(0.75)
                                .finallyDo(() -> {
                                    conveyor.stop();
                                    //elevator.setTargetHeight(ElevatorConstants.HOME_HEIGHT);
                                    //pivot.setTargetAngle(PivotConstants.HOME_ANGLE);
                            }))));
        }
    }