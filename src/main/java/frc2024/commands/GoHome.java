// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;

public class GoHome extends Command {
  boolean pivotFirst;
  Pivot pivot;
  Elevator elevator;
  Conveyor conveyor;
  Intake intake;

  Timer wait = new Timer();

  public GoHome(boolean pivotFirst, Pivot pivot, Elevator elevator, Conveyor conveyor, Intake intake) {
    addRequirements(pivot, elevator, conveyor, intake);
    this.pivotFirst = pivotFirst;
    this.pivot = pivot;
    this.elevator = elevator;
    this.conveyor = conveyor;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.stop();
    intake.stop();
    if(pivotFirst){ 
      wait.reset();
      wait.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setTargetAngle(PivotConstants.HOME_ANGLE);
    if(pivotFirst){
      if(wait.hasElapsed(0.1)) {
        elevator.setTargetHeight(ElevatorConstants.HOME_HEIGHT);
      }
    } else {
      elevator.setTargetHeight(ElevatorConstants.HOME_HEIGHT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //pivot.resetToAbsolute();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.getPivotAtTarget().getAsBoolean() && elevator.getElevatorAtTarget().getAsBoolean();
  }
}
