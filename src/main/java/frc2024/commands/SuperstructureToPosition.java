package frc2024.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.SuperstructureState;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;

public class SuperstructureToPosition extends SequentialCommandGroup{

    public SuperstructureToPosition(SuperstructureState position, Elevator elevator, Pivot pivot){
        setName("SuperstructureToPosition");
        addCommands(
            elevator.heightCommand(position.elevatorPosition)
                .alongWith(pivot.angleCommand(position.pivotAngle))
        );
    }
}
