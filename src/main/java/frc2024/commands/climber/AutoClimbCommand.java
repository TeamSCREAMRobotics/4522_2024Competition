// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ClimberConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.subsystems.Climber;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;

public class AutoClimbCommand extends Command {

  Climber climber;
  Elevator elevator;
  Pivot pivot;

  boolean isTrapping;

  public AutoClimbCommand(Climber climber, Elevator elevator, Pivot pivot, boolean isTrapping) {
    addRequirements(climber, elevator, pivot);

    this.climber = climber;
    this.elevator = elevator;
    this.pivot = pivot;
    this.isTrapping = isTrapping;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setTargetHeight(ClimberConstants.CLIMBER_TOP);
    elevator.setTargetHeight(ElevatorConstants.ELEVATOR_TRAP_CHAIN_POSITION);
    pivot.setTargetAngle(PivotConstants.PIVOT_TRAP_CHAIN_ANGLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setTargetHeight(ClimberConstants.CLIMBER_BOTTOM);

    if(isTrapping){
      elevator.setTargetHeight(ElevatorConstants.ELEVATOR_TRAP_CHAIN_POSITION);
      pivot.setTargetAngle(PivotConstants.PIVOT_TRAP_CHAIN_ANGLE);
    }
    else {
      elevator.setTargetHeight(ElevatorConstants.ELEVATOR_HOME_POSITION);
      pivot.setTargetAngle(PivotConstants.PIVOT_HOME_ANGLE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getElevatorAtTarget() && pivot.getPivotAtTarget() && climber.getClimberAtTarget();
  }
}
