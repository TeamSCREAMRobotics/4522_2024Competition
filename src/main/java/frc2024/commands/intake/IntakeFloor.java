// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.intake;

import java.util.function.BooleanSupplier;

import com.team4522.lib.util.COTSFalconSwerveConstants.driveGearRatios;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.commands.SuperstructureToPosition;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;

public class IntakeFloor extends SequentialCommandGroup {
  
  public IntakeFloor(Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake, LED led, BooleanSupplier endGame) {
    addCommands(
        elevator.heightCommand(endGame.getAsBoolean() ? ElevatorConstants.HOME_HEIGHT_ENDGAME : ElevatorConstants.HOME_HEIGHT)
            .alongWith(pivot.angleCommand(endGame.getAsBoolean() ? PivotConstants.HOME_ANGLE_ENDGAME : PivotConstants.HOME_ANGLE))
            .alongWith(intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT))
            .alongWith(conveyor.dutyCycleCommand(endGame.getAsBoolean() ? ConveyorConstants.AMP_OUTPUT+0.25 : ConveyorConstants.TRANSFER_OUTPUT))
            .alongWith(led.strobeCommand(Color.kRed, 0.5))
      );
  }
}
