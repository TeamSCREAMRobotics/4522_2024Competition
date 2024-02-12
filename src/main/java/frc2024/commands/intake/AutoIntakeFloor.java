// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.intake;

import java.util.concurrent.locks.Condition;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.ElevatorPivotPosition;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.commands.SuperstructureToPosition;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.IntakePipeline;
import frc2024.subsystems.Vision.LEDMode;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeFloor extends SequentialCommandGroup {
  public AutoIntakeFloor(DoubleSupplier[] translation, Swerve swerve, Elevator elevator, Pivot pivot, Intake intake, Conveyor conveyor) {
    addCommands(
        new FaceVisionTarget(
            swerve, 
            Vision.getTV(Limelight.INTAKE) 
                ? new DoubleSupplier[]{() -> 0, () -> 3} 
                : translation, 
            SwerveConstants.VISION_ROTATION_CONSTANTS, 
            Limelight.INTAKE)
                .alongWith(
                    new IntakeFloor(elevator, pivot, conveyor, intake)
                        .onlyWhile(() -> Vision.getTY(Limelight.INTAKE) < VisionConstants.AUTO_INTAKE_Y_THRESHOLD)
                )
            .finallyDo((interrupted) -> new SuperstructureToPosition(ElevatorPivotPosition.HOME, elevator, pivot))
    );
  }

  public AutoIntakeFloor(Swerve swerve, Elevator elevator, Pivot pivot, Intake intake, Conveyor conveyor) {
    addCommands(
        new FaceVisionTarget(
            swerve, 
            Vision.getTV(Limelight.INTAKE) 
                ? new DoubleSupplier[]{() -> 0, () -> 3} 
                : new DoubleSupplier[]{() -> 0, () -> 0}, 
            SwerveConstants.VISION_ROTATION_CONSTANTS, 
            Limelight.INTAKE)
                .alongWith(
                    new IntakeFloor(elevator, pivot, conveyor, intake)
                        .onlyWhile(() -> Vision.getTY(Limelight.INTAKE) < VisionConstants.AUTO_INTAKE_Y_THRESHOLD)
                )
            .finallyDo((interrupted) -> new SuperstructureToPosition(ElevatorPivotPosition.HOME, elevator, pivot))
    );
  }
}