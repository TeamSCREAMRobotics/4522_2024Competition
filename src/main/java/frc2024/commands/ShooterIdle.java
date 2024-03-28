// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import java.util.function.BooleanSupplier;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class ShooterIdle extends Command {
  
  BooleanSupplier endgame;
  Swerve swerve;
  Conveyor conveyor;
  Shooter shooter;

  Translation2d target;

  public ShooterIdle(BooleanSupplier endgame, Swerve swerve, Conveyor conveyor, Shooter shooter) {
    addRequirements(shooter);
    this.endgame = endgame;
    this.swerve = swerve;
    this.conveyor = conveyor;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = AllianceFlipUtil.getTargetSpeaker().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(conveyor.hasPiece(false).getAsBoolean() 
       && ScreamUtil.calculateDistanceToTranslation(swerve.getPose().getTranslation(), target) <= Units.feetToMeters(25.0)
       && RobotContainer.getCurrentState().get() != SuperstructureState.AMP){
      if(!endgame.getAsBoolean()){
        shooter.setTargetVelocity(3000);
      } else {
        shooter.stop();
      }
    } else {
      shooter.setTargetVelocity(ShooterConstants.IDLE_VELOCITY);
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
