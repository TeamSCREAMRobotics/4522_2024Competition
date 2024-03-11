package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.PIDConstants;
import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class AutoShootSequence extends Command{

    Swerve swerve;
    Elevator elevator;
    Pivot pivot;
    Shooter shooter;
    Conveyor conveyor;
    Timer timeout = new Timer();
    boolean shouldTimeout;
    DoubleSupplier[] translation;

    PIDController rotationController;

    public AutoShootSequence(DoubleSupplier[] translationSup, boolean timeout, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor){
        addRequirements(swerve, elevator, pivot, shooter, conveyor);
        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shouldTimeout = timeout;
        this.translation = translationSup;
        rotationController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    }

    public AutoShootSequence(boolean timeout, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor){
        addRequirements(swerve, elevator, pivot, shooter, conveyor);
        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shouldTimeout = timeout;
        this.translation = null;
        rotationController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    }

    @Override
    public void initialize() {
        timeout.reset();
        timeout.start();
    }

    @Override
    public void execute() {
        double rotationValue = Math.abs(Vision.getTX(Limelight.SHOOTER)) < 2.0 ? 0 : rotationController.calculate(Vision.getTX(Limelight.SHOOTER), 0.0);
        Translation2d translationValue = translation == null ? new Translation2d() : new Translation2d(translation[0].getAsDouble(), translation[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
        swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(translationValue, rotationValue));

        shooter.setTargetVelocity(AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).velocityRPM());
        pivot.setTargetAngle(AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).pivotAngle());

        if((shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean() && shooter.getRPM() > ShooterConstants.TARGET_THRESHOLD && Vision.getLockedToTarget(Limelight.SHOOTER)) || timeout.hasElapsed(2.0)){
            conveyor.setConveyorOutput(ConveyorConstants.SHOOT_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetVelocity(ShooterConstants.IDLE_VELOCITY);
        pivot.stop();
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return !conveyor.hasPiece(false).getAsBoolean() || (timeout.hasElapsed(2.5) && shouldTimeout);
    }
    
}
