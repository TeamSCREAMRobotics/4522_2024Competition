// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeFloor extends SequentialCommandGroup {
    public AutoIntakeFloor(Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake, LED led){
        setName("AutoIntakeFloor");

        addCommands(
            conveyor.dutyCycleCommand(ConveyorConstants.TRANSFER_OUTPUT)
            .alongWith(intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT))
            .alongWith(elevator.heightCommand(ElevatorConstants.HOME_HEIGHT))
            .alongWith(pivot.angleCommand(PivotConstants.HOME_ANGLE))
                .until(conveyor.hasPiece(false))
                .finallyDo(() -> {
                    //if(!interrupted){
                        intake.stop();
                        conveyor.stop();
                    //}
                })
        );
    }
}