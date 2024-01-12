// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.subsystems.swerve.Swerve;

public class TrackVisionTarget extends Command {
  /** Creates a new TrackVisionTarget. */
  Swerve swerve;
  ScreamPIDConstants testConstants = new ScreamPIDConstants(5, 0, 0);
  public TrackVisionTarget(Swerve swerve) {
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setChassisSpeeds(
      swerve.visionTargetSpeeds(LimelightHelpers.getTV("limelight-front"),LimelightHelpers.getTX("limelight-front"), LimelightHelpers.getTY("limelight-front"), testConstants, testConstants)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
