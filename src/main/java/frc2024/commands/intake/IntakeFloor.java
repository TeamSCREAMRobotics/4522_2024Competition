// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.Position;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.commands.SuperstructureToPosition;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;

public class IntakeFloor extends SequentialCommandGroup {
  public IntakeFloor(Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake) {
    addCommands(
        elevator.positionCommand(ElevatorConstants.ELEVATOR_HOME_POSITION)
            .alongWith(pivot.angleCommand(PivotConstants.PIVOT_HOME_ANGLE))
            .alongWith(intake.outputCommand(IntakeConstants.INTAKE_SPEED))
            .andThen(conveyor.outputCommand(ConveyorConstants.TRANSFER_SPEED))
            .finallyDo((interrupted) -> new SuperstructureToPosition(Position.HOME, elevator, pivot))
    );
  }
}