// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.SwerveConstants;
import frc2024.commands.swerve.FaceVisionTargetCommand;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.IntakePipeline;
import frc2024.subsystems.Vision.LEDMode;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickupCommand extends SequentialCommandGroup {
  /** Creates a new DriveToAndPickup. */
  public AutoPickupCommand(Swerve swerve) {
    addRequirements(swerve);

    addCommands(
      new ParallelRaceGroup(
        new FaceVisionTargetCommand(
          swerve, 
          () -> new Translation2d(0, 0),
          SwerveConstants.VISION_ROTATION_CONSTANTS,
          IntakePipeline.DETECTOR_RIGHTMOST
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(1), 
          Commands.runOnce(() -> Vision.setLEDMode(Limelight.INTAKE, LEDMode.ON)))
          .andThen(Commands.runOnce(() -> Vision.setLEDMode(Limelight.INTAKE, LEDMode.OFF))
        )
      )
    );
  }
}