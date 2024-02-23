package frc2024.auto;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team4522.lib.util.PathSequence;
import com.team4522.lib.util.PathSequence.Side;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.commands.AutoFire;
import frc2024.commands.intake.AutoIntakeFloor;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class Routines {

    private static final Timer autoTimer = new Timer();
    private static PathSequence currentSequence;
    private static final PathSequence Close4 = new PathSequence(Side.AMP, "Close4_1", "Close4_2", "Close4_3");
    private static final PathSequence AmpSide5 = new PathSequence(Side.AMP, "Close4_1", "Close4_2", "Amp6_0", "Amp6_1", "Amp6_2");
    private static final PathSequence AmpSide6 = new PathSequence(Side.AMP, "Close4_1", "Close4_2", "Amp6_0", "Amp6_1", "Amp6_2", "Amp6_3", "Amp6_4");
    private static final PathSequence SourceSide4 = new PathSequence(Side.SOURCE, "SourceSide4_1", "SourceSide4_2", "SourceSide4_3", "SourceSide4_4", "SourceSide4_5", "SourceSide4_6");

    private static Command startTimer(){
        return Commands.runOnce(
            () -> {
                autoTimer.reset();
                autoTimer.start();
            });
    }

    private static Command printTimer(){
        return Commands.runOnce(() -> {
                autoTimer.stop();
                System.out.println("[Auto] Time elapsed:  " + autoTimer.get());
            });
    }

    private static PathSequence getCurrentSequence(){
        return currentSequence;
    }

    /* public static Command testAuto(Swerve swerve){
        correctionHelper = new PathCorrectionHelper();
        return new SequentialCommandGroup(
            resetPoseCommand(swerve, "Close4_1"),
            new SequentialCommandGroup(
                correctionHelper.getCommand(0)
            )
        );
    } */
 
    public static Command Close4(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor){
        currentSequence = Close4;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Close4.getStartingPose()),
            new AutoFire(swerve, shooter, elevator, pivot, conveyor, () -> false)
                .alongWith(new FaceVisionTarget(swerve, new DoubleSupplier[] {() -> 0, () -> 0}, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                    .withTimeout(2)
                .alongWith(new WaitCommand(0.5)
                    .andThen(conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED).until(() -> !conveyor.hasPiece().getAsBoolean()).finallyDo(() -> {
                        elevator.heightCommand(ElevatorConstants.HOME_HEIGHT);
                        pivot.angleCommand(PivotConstants.HOME_ANGLE);
                    }))),
            Close4.getStart(),
            new AutoFire(swerve, shooter, elevator, pivot, conveyor, () -> false)
                .alongWith(new FaceVisionTarget(swerve, new DoubleSupplier[] {() -> 0, () -> 0}, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                    .withTimeout(2)
                .alongWith(new WaitCommand(0.5)
                    .andThen(conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED).until(() -> !conveyor.hasPiece().getAsBoolean()).finallyDo(() -> {
                        elevator.heightCommand(ElevatorConstants.HOME_HEIGHT);
                        pivot.angleCommand(PivotConstants.HOME_ANGLE);
                    }))),
            Close4.getNext(),
            new AutoFire(swerve, shooter, elevator, pivot, conveyor, () -> false)
                .alongWith(new FaceVisionTarget(swerve, new DoubleSupplier[] {() -> 0, () -> 0}, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                    .withTimeout(2)
                .alongWith(new WaitCommand(0.5)
                    .andThen(conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED).until(() -> !conveyor.hasPiece().getAsBoolean()).finallyDo(() -> {
                        elevator.heightCommand(ElevatorConstants.HOME_HEIGHT);
                        pivot.angleCommand(PivotConstants.HOME_ANGLE);
                    }))),
            Close4.getEnd(),
            new AutoFire(swerve, shooter, elevator, pivot, conveyor, () -> false)
                .alongWith(new FaceVisionTarget(swerve, new DoubleSupplier[] {() -> 0, () -> 0}, SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))
                    .withTimeout(2)
                .alongWith(new WaitCommand(0.5)
                    .andThen(conveyor.outputCommand(ConveyorConstants.SHOOT_SPEED).until(() -> !conveyor.hasPiece().getAsBoolean()).finallyDo(() -> {
                        elevator.heightCommand(ElevatorConstants.HOME_HEIGHT);
                        pivot.angleCommand(PivotConstants.HOME_ANGLE);
                    }))),
            printTimer()
        );
    }

    public static Command AmpSide6(Swerve swerve, Elevator elevator, Pivot pivot, Intake intake, Conveyor conveyor){
        currentSequence = AmpSide6;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(AmpSide6.getStartingPose()),
            AmpSide6.getStart(),
            new WaitCommand(0.5),
            AmpSide6.getNext(),
            new WaitCommand(0.5),
            AmpSide6.getNext(),
            new WaitCommand(0.5),
            AmpSide6.getNext(),
            //new AutoIntakeFloor(swerve, elevator, pivot, intake, conveyor),
            AmpSide6.getNext(),
            new WaitCommand(0.5),
            AmpSide6.getNext(),
            //new AutoIntakeFloor(swerve, elevator, pivot, intake, conveyor),
            AmpSide6.getEnd(),
            printTimer()
        );
    }

    public static Command SourceSide4(Swerve swerve){
        currentSequence = SourceSide4;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(SourceSide4.getStartingPose()),
            SourceSide4.getStart(),
            // new WaitCommand(0.75),
            SourceSide4.getNext(),
            SourceSide4.getNext(),
            // new WaitCommand(0.75),
            SourceSide4.getNext(),
            SourceSide4.getNext(),
            // new WaitCommand(0.75),
            SourceSide4.getEnd(),
            printTimer()
        );
    }

    public static Command AmpSide5(Swerve swerve){
        return new SequentialCommandGroup(
        startTimer(),
        swerve.resetPoseCommand(AmpSide6.getStartingPose()),
        AmpSide6.getStart(),
        new WaitCommand(0.5),
        AmpSide6.getNext(),
        new WaitCommand(0.5),
        AmpSide6.getNext(),
        new WaitCommand(0.5),
        AmpSide6.getNext(),
        AmpSide6.getEnd(),
        new WaitCommand(0.5),
        printTimer()
        );
    }
}
