package frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorPivotPosition;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Shooter;

public class AutoScore extends SequentialCommandGroup{
    
    private ElevatorPivotPosition currentPosition;

    public AutoScore(ElevatorPivotPosition position, Conveyor conveyor){
        currentPosition = position;
    }

    public Command getScoreCommand(ElevatorPivotPosition position, Conveyor conveyor){
        Command command;
        switch (position) {
            case AMP:
            case TRAP_CHAIN:
                command = conveyor.outputCommand(ConveyorConstants.AMP_TRAP_OUTPUT);
                break;
            case CHAIN:
            case PODIUM_DEFENDED:
            case PODIUM:
            case SUBWOOFER:
            case SUBWOOFER_DEFENDED:
                command = conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED);
                break;
            default:
                command = conveyor.stopCommand();
                break;
        }
        return command;
    }
}
