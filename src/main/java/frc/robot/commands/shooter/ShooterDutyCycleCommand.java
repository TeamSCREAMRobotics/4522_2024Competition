// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.shuffleboard.tabs.ShooterTab;
import frc.robot.subsystems.Shooter;

public class ShooterDutyCycleCommand extends Command {
  
  Shooter shooter;
  double dutyCycle;

  public ShooterDutyCycleCommand(Shooter shooter, double dutyCycle) {
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
    dutyCycle = ShuffleboardConstants.UPDATE_SHOOTER && dutyCycle != 0.0 ? ShooterTab.getShooterDutyCycle() : dutyCycle;
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
