package frc2024.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.RobotContainer;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;

public class GoHome extends SequentialCommandGroup{

    public GoHome(boolean shouldWait, Pivot pivot, Elevator elevator, Conveyor conveyor, Intake intake){
        addRequirements(pivot, elevator, conveyor, intake);
        addCommands(
            new WaitCommand(shouldWait ? 0.15 : 0).deadlineWith(pivot.angleCommand(PivotConstants.HOME_ANGLE))
                .andThen(elevator.heightCommand(ElevatorConstants.HOME_HEIGHT))
                .alongWith(conveyor.stopCommand())
                .alongWith(intake.stopCommand())
        );
    }
}