// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.subsystems.Conveyor;

public class ConveyorCommand extends Command {
  
  Conveyor conveyor;
  double speed;
  boolean beamStop;

  public ConveyorCommand(Conveyor conveyor, double speed, boolean beamStop) {
    addRequirements(conveyor);

    this.conveyor = conveyor;
    this.speed = speed;
    this.beamStop = beamStop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.setConveyorOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conveyor.hasPiece() && beamStop;
  }
}