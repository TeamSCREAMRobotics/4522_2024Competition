package frc2024.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.RobotContainer;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;

public class SuperstructureToPosition extends SequentialCommandGroup{

    public SuperstructureToPosition(SuperstructureState position, Elevator elevator, Pivot pivot){
        addCommands(
            elevator.heightCommand(position.elevatorPosition)
                .alongWith(pivot.angleCommand(position.pivotAngle))
        );
    }
}
