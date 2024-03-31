// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;

public class Rehome extends Command {

  Elevator elevator;
  Pivot pivot;

  public Rehome(Elevator elevator, Pivot pivot) {
    addRequirements(elevator, pivot);
    setName("Rehome");

    this.elevator = elevator;
    this.pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.voltageCommand(ElevatorConstants.REHOME_VOLTAGE);
    pivot.dutyCycleCommand(PivotConstants.AUTO_ZERO_OUTPUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      elevator.zeroPosition();
      pivot.zeroPivot();
    }
    elevator.stop();
    pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean elevatorCurrent = elevator.getElevatorCurrent() > 3.0;
    boolean pivotCurrent = pivot.getPivotCurrent() > 3.0;
    
    return elevatorCurrent && pivotCurrent;
  }
}
