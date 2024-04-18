// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.StabilizerConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Stabilizers;
import frc2024.subsystems.swerve.Swerve;

public class ClimbSequence extends Command {
  DoubleSupplier[] translation;
  DoubleSupplier rotation;
  DoubleSupplier elevatorVoltage;

  Swerve swerve;
  Elevator elevator;
  Pivot pivot;
  Stabilizers stabilizers;
  LED led;

  boolean manualMode = false;
  int index = -1;

  public ClimbSequence(DoubleSupplier[] translation, DoubleSupplier rotation, DoubleSupplier elevatorVoltage, Swerve swerve, Elevator elevator, Pivot pivot, Stabilizers stabilizers, LED led) {
    addRequirements(swerve, elevator, pivot, stabilizers, led);
    this.translation = translation;
    this.rotation = rotation;
    this.elevatorVoltage = elevatorVoltage;
    this.swerve = swerve;
    this.elevator = elevator;
    this.pivot = pivot;
    this.stabilizers = stabilizers;
    this.led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.rainbow(10, 1.5);
    Translation2d translation = new Translation2d(this.translation[0].getAsDouble(), this.translation[1].getAsDouble()).times((SwerveConstants.MAX_SPEED / 2.0));
    double rotation = this.rotation.getAsDouble() * (SwerveConstants.MAX_ANGULAR_VELOCITY / 2.0);
    System.out.println(manualMode + " " + elevatorVoltage.getAsDouble());
    switch(index){
      case 0:
        swerve.setChassisSpeeds(swerve.robotRelativeSpeeds(translation, rotation));
        if(elevator.getElevatorHeight() > ElevatorConstants.TRAP_CHAIN_HEIGHT - 0.5){
          manualMode = true;
        }
        if(manualMode){
          elevator.setElevatorVoltage(Math.abs(elevatorVoltage.getAsDouble()) > 1.25 ? elevatorVoltage.getAsDouble() : 0);
        } else {
          elevator.setTargetHeight(ElevatorConstants.TRAP_CHAIN_HEIGHT);
        }
        pivot.setTargetAngle(PivotConstants.HOME_ANGLE);
        stabilizers.setTargetPosition(StabilizerConstants.DOWN_POSITION);
      break;
      case 1:
        swerve.stopAll();
        elevator.setTargetHeight(ElevatorConstants.TRAP_CHAIN_HEIGHT);
        pivot.setTargetAngle(PivotConstants.TRAP_CHAIN_ANGLE);
      break;
      default:
        swerve.stopAll();
        elevator.stop();
        pivot.stop();
        stabilizers.stop();
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index = -1;
    manualMode = false;
    swerve.stopAll();
    elevator.stop();
    pivot.stop();
    stabilizers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index == 2;
  }

  public void advance(){
    index ++;
  }
}
