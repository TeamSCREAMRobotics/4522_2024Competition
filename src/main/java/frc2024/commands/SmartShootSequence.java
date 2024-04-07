package frc2024.commands;

import java.util.function.DoubleSupplier;

import com.team1706.SmartShooting;
import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class SmartShootSequence extends Command{

    Swerve swerve;
    Elevator elevator;
    Pivot pivot;
    Shooter shooter;
    Conveyor conveyor;
    LED led;

    Timer timeout = new Timer();
    boolean shouldTimeout;
    boolean fire;
    DoubleSupplier[] translation;
    boolean virtualCalculation;

    PIDController rotationController;

    public SmartShootSequence(DoubleSupplier[] translationSup, boolean timeout, boolean fire, boolean virtualCalculation, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, LED led){
        addRequirements(swerve, elevator, pivot, shooter);
        setName("SmartShootSequence");

        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.led = led;
        this.shouldTimeout = timeout;
        this.fire = fire;
        this.virtualCalculation = virtualCalculation;
        this.translation = translationSup;
        rotationController = SwerveConstants.VISION_ROTATION_CONSTANTS.toPIDController(); //SwerveConstants.VISION_MOVING_ROTATION_CONSTANTS.toPIDController();
    }

    public SmartShootSequence(boolean timeout, boolean virtualCalculation, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, LED led){
        addRequirements(swerve, elevator, pivot, shooter);

        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.led = led;
        this.shouldTimeout = timeout;
        this.virtualCalculation = virtualCalculation;
        this.translation = null;
        rotationController = SwerveConstants.VISION_ROTATION_CONSTANTS.toPIDController(); //SwerveConstants.VISION_MOVING_ROTATION_CONSTANTS.toPIDController();
    }

    @Override
    public void initialize() {
        timeout.reset();
        timeout.start();
    }

    @Override
    public void execute() {
        /* Sets the direction coefficient */
        double directionCoefficient = AllianceFlipUtil.getDirectionCoefficient();
        /* sets the physical target to the speaker based on the alliance */
        Translation2d physicalTarget = AllianceFlipUtil.getTargetSpeaker().getTranslation().plus(new Translation2d(Units.inchesToMeters(12.0) * directionCoefficient, Units.inchesToMeters(8) * directionCoefficient));
        
        /* sets the target rotation value for the physical speaker */
        double rotationValue = SmartShooting.getRotationToPoint(swerve, pivot, shooter, physicalTarget, false, false, rotationController);

        /* Sets the the driving translation value */
        Translation2d translationValue = translation == null ? new Translation2d() : new Translation2d(translation[0].getAsDouble(), translation[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlipUtil.getDirectionCoefficient());
        
        if(virtualCalculation){
            /* Scales the driving translation value to slow the drivetrain while aiming */
            translationValue = translationValue.times(SwerveConstants.SHOOT_WHILE_MOVING_SCALAR);

            /* Sets the virtual target */
            Translation2d virtualTarget = SmartShooting.calculateVirtualTarget(swerve, pivot, shooter, physicalTarget);

            /* Sets the new target rotation value based on the virtual target */
            rotationValue = SmartShooting.getRotationToPoint(swerve, pivot, shooter, virtualTarget, false, false, rotationController); //virtualCalculation is false because we are passing in the virtualTarget already

            /* Sets the shooterRPMs and pivot angle based on the virtual target */
            shooter.setTargetVelocity(MathUtil.clamp(SmartShooting.calculateShotTrajectory(() -> elevator.getElevatorHeight(), SmartShooting.getDistanceToTarget_SHOOTER(swerve, virtualTarget, FieldConstants.SPEAKER_OPENING_HEIGHT)).velocityRPM(), 3250.0, ShooterConstants.SHOOTER_MAX_VELOCITY));
            pivot.setTargetAngle(SmartShooting.calculateShotTrajectory(() -> elevator.getElevatorHeight(), SmartShooting.getDistanceToTarget_SHOOTER(swerve, virtualTarget, FieldConstants.SPEAKER_OPENING_HEIGHT)).pivotAngle());
        } else{
            /* Sets the shooterRPMs and pivot based on the physical target */
            shooter.setTargetVelocity(SmartShooting.calculateShotTrajectory(() -> elevator.getElevatorHeight(), SmartShooting.getDistanceToTarget_SHOOTER(swerve, physicalTarget, FieldConstants.SPEAKER_OPENING_HEIGHT)).velocityRPM());
            pivot.setTargetAngle(SmartShooting.calculateShotTrajectory(() -> elevator.getElevatorHeight(), SmartShooting.getDistanceToTarget_SHOOTER(swerve, physicalTarget, FieldConstants.SPEAKER_OPENING_HEIGHT)).pivotAngle());
        }

        /* Sets the chassis speeds based on the set translation value and rotation value */
        swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(translationValue, rotationValue));
        /* Changes led color based on shooterRPMs */
        led.scaledTarget(Color.kDeepSkyBlue /* Color.kGoldenrod */, shooter.getRPM(), shooter.getTargetVelocity());

        /* Checks if all conditions are met to fire the shot */
        if(fire && (shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean() && shooter.getRPM() > ShooterConstants.TARGET_THRESHOLD) || (timeout.hasElapsed(1) && shouldTimeout)){
          conveyor.setConveyorOutput(ConveyorConstants.SHOOT_OUTPUT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        /* Sets the shooterRPMs back to idle and tells the pivot and conveyor to stop */
        shooter.setTargetVelocity(ShooterConstants.IDLE_VELOCITY);
        pivot.stop();
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        /* Checks wether or not to end the command */
        return !conveyor.hasPiece(false).getAsBoolean() || (timeout.hasElapsed(2.0) && shouldTimeout);
    }
}
