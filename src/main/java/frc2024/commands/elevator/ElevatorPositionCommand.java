// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.subsystems.Elevator;

public class ElevatorPositionCommand extends Command {
  
  Elevator elevator;
  double height;

  public ElevatorPositionCommand(Elevator elevator, double height) {
    addRequirements(elevator);

    this.elevator = elevator;
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setTargetHeight(height);
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