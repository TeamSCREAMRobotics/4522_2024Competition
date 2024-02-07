package frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;

public class GoHome extends SequentialCommandGroup{

    public GoHome(Elevator elevator, Pivot pivot){
        addCommands(
            elevator.positionCommand(ElevatorConstants.ELEVATOR_HOME_POSITION)
                .alongWith(pivot.angleCommand(PivotConstants.PIVOT_HOME_ANGLE))
                .until(() -> endCondition(elevator, pivot))
        );
    }

    public boolean endCondition(Elevator elevator, Pivot pivot){
        return elevator.getElevatorAtTarget() && pivot.getPivotAtTarget();
    }
}
