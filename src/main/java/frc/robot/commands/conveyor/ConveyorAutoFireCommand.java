// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class ConveyorAutoFireCommand extends Command {
  
  Conveyor conveyor;
  Shooter shooter;
  Pivot pivot;
  Elevator elevator;

  public ConveyorAutoFireCommand(Conveyor conveyor, Shooter shooter, Pivot pivot, Elevator elevator) {
    addRequirements(conveyor, shooter, pivot, elevator);

    this.conveyor = conveyor;
    this.shooter = shooter;
    this.pivot = pivot;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean shooterAtSpeed = shooter.getWheelRPM() > ShooterConstants.AUTO_SHOOT_VELOCITY_THRESHOLD;
    boolean pivotAtAngle = pivot.getPivotAtTarget();
    boolean elevatorAtPosition = elevator.getElevatorAtTarget();

    if(shooterAtSpeed && pivotAtAngle && elevatorAtPosition){
      conveyor.setConveyor(new DutyCycleOut(ConveyorConstants.SPEAKER_SPEED));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
