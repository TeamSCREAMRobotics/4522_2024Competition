package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.team1706.SmartShooting;
import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class SmartShootSequence extends Command{

    Swerve swerve;
    Elevator elevator;
    Pivot pivot;
    Shooter shooter;
    Conveyor conveyor;
    Timer timeout = new Timer();
    boolean shouldTimeout;
    DoubleSupplier[] translation;
    boolean virtualCalculation;

    PIDController rotationController;

    public SmartShootSequence(DoubleSupplier[] translationSup, boolean timeout, boolean virtualCalculation, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor){
        addRequirements(swerve, elevator, pivot, shooter, conveyor);

        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shouldTimeout = timeout;
        this.virtualCalculation = virtualCalculation;
        this.translation = translationSup;
        rotationController = SwerveConstants.SNAP_CONSTANTS.toPIDController();
    }

    public SmartShootSequence(boolean timeout, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor){
        addRequirements(swerve, elevator, pivot, shooter, conveyor);

        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.shouldTimeout = timeout;
        this.virtualCalculation = false;
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
        
        if(virtualCalculation){
            Translation2d physicalTarget = AllianceFlippable.getTargetSpeaker().getTranslation();
            rotationValue = SmartShooting.getRoatationToPoint(swerve, pivot, shooter, physicalTarget, virtualCalculation, false);
            translationValue.times(SwerveConstants.SHOOT_WHILE_MOVING_SCALAR);

            Translation2d virtualTarget = SmartShooting.calculateVirtualTarget(swerve, pivot, shooter, physicalTarget);

            shooter.setTargetVelocity(SmartShooting.calculateShotTrajectory(() -> elevator.getElevatorHeight(), SmartShooting.getDistanceToTarget_SHOOTER(swerve, virtualTarget, FieldConstants.SPEAKER_TAG_HEIGHT)).velocityRPM());
        } else{
            shooter.setTargetVelocity(AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).velocityRPM());
        }

        swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(translationValue, rotationValue));
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
