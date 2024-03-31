package frc2024.commands.auto;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.PIDConstants;
import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.commands.AutoFire;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.LED;
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
    LED led;
    Timer timeout = new Timer();
    boolean shouldTimeout;

    PIDController rotationController;

    public AutoShootSequence(boolean shouldTimeout, Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, LED led){
        addRequirements(swerve, elevator, pivot, shooter);
        setName("AutoShootSequence");
        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.led = led;
        this.shouldTimeout = shouldTimeout;
        rotationController = SwerveConstants.VISION_ROTATION_CONSTANTS.toPIDController();
    }

    @Override
    public void initialize() {
        timeout.reset();
        timeout.start();
    }

    @Override
    public void execute() {
        double rotationValue = Math.abs(Vision.getTX(Limelight.SHOOT_SIDE)) < 2.0 ? 0 : rotationController.calculate(Vision.getTX(Limelight.SHOOT_SIDE), 0.0);
        swerve.setChassisSpeeds(swerve.fieldRelativeSpeeds(new Translation2d(), rotationValue));

        if(Vision.getTV(Limelight.SHOOT_SIDE)){
            shooter.setTargetVelocity(AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).velocityRPM());
            pivot.setTargetAngle(AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).pivotAngle());
            led.scaledTarget(Color.kGoldenrod, shooter.getRPM(), shooter.getTargetVelocity());
        }

        if((shooter.getShooterAtTarget().getAsBoolean() && pivot.getPivotAtTarget().getAsBoolean() && shooter.getRPM() > ShooterConstants.TARGET_THRESHOLD && Vision.getTV(Limelight.SHOOT_SIDE)) || (timeout.hasElapsed(2) && shouldTimeout)){
            conveyor.setConveyorOutput(ConveyorConstants.SHOOT_OUTPUT);
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
