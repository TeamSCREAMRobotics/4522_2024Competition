// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

  Intake intake;
  double speed;
  boolean autoDetect;

  public IntakeCommand(Intake intake, double speed, boolean autoDetect) {
    addRequirements(intake);
    this.intake = intake;
    this.speed = speed;
    this.autoDetect = autoDetect;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(autoDetect){
      intake.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoDetect && Conveyor.hasPiece();
  }
}
