// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.swerve.FaceVisionTargetCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.IntakePipeline;
import frc.robot.subsystems.Vision.LEDMode;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.swerve.Swerve;

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
          new DoubleSupplier[]{
            () -> 1,
            () -> 0
          }, 
          SwerveConstants.VISION_ROTATION_CONSTANTS,
          IntakePipeline.DETECTOR_RIGHTMOST
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(1), 
          new InstantCommand(() -> Vision.setLEDMode(Limelight.INTAKE, LEDMode.ON)))
          .andThen(new InstantCommand(() -> Vision.setLEDMode(Limelight.INTAKE, LEDMode.OFF))
        )
      )
    );
  }
}
