package frc2024.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;

public class ShootSequence extends Command{

    Elevator elevator;
    Pivot pivot;
    Shooter shooter;
    Conveyor conveyor;
    SuperstructureState targetState;
    double velocity;

    public ShootSequence(SuperstructureState state, double velocity, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor){
        addRequirements(elevator, pivot, shooter, conveyor);
        setName("ShootSequence");
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.targetState = state;
        this.velocity = velocity;
    }

    @Override
    public void execute() {
        shooter.setTargetVelocity(velocity);
        pivot.setTargetAngle(targetState.pivotAngle);
        elevator.setTargetHeight(targetState.elevatorPosition);

        if(shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean() && shooter.getRPM() > ShooterConstants.TARGET_THRESHOLD){
            conveyor.setConveyorOutput(ConveyorConstants.SHOOT_OUTPUT);
        }

        System.out.println(shooter.getShooterAtTarget().getAsBoolean() + " " + pivot.getPivotAtTarget().getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        elevator.setTargetHeight(ElevatorConstants.HOME_HEIGHT);
        pivot.setTargetAngle(PivotConstants.HOME_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return !conveyor.hasPiece(false).getAsBoolean();
    }
}