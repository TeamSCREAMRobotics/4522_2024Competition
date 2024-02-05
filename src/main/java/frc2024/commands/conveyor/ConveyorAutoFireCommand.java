// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.conveyor;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.team1706.SmartShooting;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class ConveyorAutoFireCommand extends Command {
  
  Swerve swerve;
  Conveyor conveyor;
  Shooter shooter;
  Pivot pivot;
  Elevator elevator;

  Translation2d target;
  double distanceToTarget;

  public ConveyorAutoFireCommand(Swerve swerve, Conveyor conveyor, Shooter shooter, Pivot pivot, Elevator elevator) {
    addRequirements(conveyor, shooter, pivot, elevator);

    this.swerve = swerve;
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.pivot = pivot;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = AllianceFlippable.Translation2d(FieldConstants.BLUE_SPEAKER_OPENING, FieldConstants.RED_SPEAKER_OPENING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToTarget = ScreamUtil.calculateDistanceToPose(swerve.getPose(), new Pose2d(SmartShooting.calculateVirtualTarget(swerve, target), new Rotation2d()));

    boolean shooterAtSpeed = shooter.getWheelRPM() > ShooterConstants.minumumShooterOutput.get(distanceToTarget);
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
