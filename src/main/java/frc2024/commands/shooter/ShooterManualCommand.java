// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ShuffleboardConstants;
import frc2024.dashboard.tabs.ShooterTab;
import frc2024.subsystems.Shooter;

public class ShooterManualCommand extends Command {
  
  Shooter shooter;
  double dutyCycle;

  public ShooterManualCommand(Shooter shooter, double dutyCycle) {
    addRequirements(shooter);

    this.shooter = shooter;
    this.dutyCycle = dutyCycle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dutyCycle = ShuffleboardConstants.UPDATE_SHOOTER ? ShooterTab.getShooterDutyCycle() : dutyCycle;
    shooter.setShooterOutput(dutyCycle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}