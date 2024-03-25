package frc2024.auto;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.PathSequence;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.PathSequence.Side;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.commands.AutoFire;
import frc2024.commands.ShootSequence;
import frc2024.commands.SmartShootSequence;
import frc2024.commands.AutoShootSequence;
import frc2024.commands.SuperstructureToPosition;
import frc2024.commands.intake.AutoIntakeFloor;
import frc2024.commands.intake.IntakeFloor;
import frc2024.commands.swerve.FacePoint;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class Routines {

    private static final Timer autoTimer = new Timer();
    private static PathSequence currentSequence;
    private static final PathSequence Amp4Close = new PathSequence(Side.AMP, "Amp4Close#1", "Amp4Close#2", "Amp4Close#3");
    private static final PathSequence Amp5_1Center = new PathSequence(Side.AMP,  "Amp5_1Center#1", "Amp5_1Center#2");
    private static final PathSequence Amp6Center = new PathSequence(Side.AMP, "Amp4Close#1", "Amp4Close#1", "Amp6Center#0", "Amp6Center#1", "Amp6Center#2", "Amp6Center#3", "Amp6Center#4");
    private static final PathSequence Source4Center = new PathSequence(Side.SOURCE, "Source4Center#1", "Source4Center#3");
    private static final PathSequence Amp4Center = new PathSequence(Side.AMP, "Amp4Center#1", "Amp4Center#2", "Amp4Center#3");
    private static final PathSequence SweepCenter = new PathSequence(Side.SOURCE, "SweepCenter#1", "SweepCenter#2");
    private static final PathSequence Sweep2_Source = new PathSequence(Side.SOURCE, "Sweep2#1", "3Sweep#2");
    private static final PathSequence Sweep3_Source = new PathSequence(Side.SOURCE, "SweepCenter#1", "3Sweep#1", "3Sweep#2");
    private static final PathSequence Amp5Center_2 = new PathSequence(Side.AMP, "Amp5Center#1", "Amp5Center#2", "Amp5Center#3", "Amp5Center#4");
    private static final PathSequence Source3_NoStage = new PathSequence(Side.SOURCE, "Source3#0", "Source3#1", "Source3#2");
    private static final PathSequence Leave = new PathSequence(Side.SOURCE, "Leave");

    private static Command startTimer(){
        return Commands.runOnce(
            () -> {
                autoTimer.reset();
                autoTimer.start();
            });
    }

    private static Command printTimer(){
        return Commands.runOnce(() -> {
                System.out.println("[Auto] Time elapsed:  " + autoTimer.get());
            });
    }

    private static PathSequence getCurrentSequence(){
        return currentSequence;
    }

    public static Command testAuto(Swerve swerve){
        PathPlannerPath path = PathPlannerPath.fromPathFile("Test");
        return new SequentialCommandGroup(
            swerve.resetPoseCommand(AllianceFlippable.MirroredPose2d(path.getPreviewStartingHolonomicPose())),
            AutoBuilder.followPath(path)
        );
    }
 
    public static Command Amp4Close(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Amp4Close;

        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp4Close.getStartingPose()),
            new ShootSequence(SuperstructureState.SUBWOOFER, ShooterConstants.SUBWOOFER_VELOCITY, elevator, pivot, shooter, conveyor),
            Amp4Close.getIndex(0),
            new AutoIntakeFloor(elevator, pivot, conveyor, intake, led),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Amp4Close.getIndex(1),
            new AutoIntakeFloor(elevator, pivot, conveyor, intake, led),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Amp4Close.getIndex(2),
            new AutoIntakeFloor(elevator, pivot, conveyor, intake, led),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer()
        );
    }

    public static Command Amp4Close_FastShootTest(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Amp4Close;

        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp4Close.getStartingPose()),
            new ShootSequence(SuperstructureState.SUBWOOFER, ShooterConstants.SUBWOOFER_VELOCITY, elevator, pivot, shooter, conveyor))
            .andThen(
                new SequentialCommandGroup(
                Amp4Close.getIndex(0),
                new WaitCommand(0.5),
                Amp4Close.getIndex(1),
                new WaitCommand(0.5),
                Amp4Close.getIndex(2),
                printTimer())
                    .alongWith(
                        swerve.overrideRotationTargetCommand(
                            () -> ScreamUtil.calculateAngleToPoint(
                                swerve.getPose().getTranslation(), 
                                AllianceFlippable.getTargetSpeaker().getTranslation()).plus(new Rotation2d(Math.PI))))
                    .alongWith(elevator.heightCommand(ElevatorConstants.HOME_HEIGHT))
                    .alongWith(intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT))
                    .alongWith(conveyor.dutyCycleCommand(ConveyorConstants.TRANSFER_OUTPUT))
                    .alongWith(shooter.velocityCommand(() -> AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).velocityRPM()))
                    .alongWith(pivot.angleCommand(() -> AutoFire.calculateShotTrajectory(() -> elevator.getElevatorHeight()).pivotAngle())));
    }

    public static Command Amp4Center(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Amp4Center;
        
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp4Center.getStartingPose()),
            //new ShootSequence(SuperstructureState.SUBWOOFER, ShooterConstants.SUBWOOFER_VELOCITY, elevator, pivot, shooter, conveyor))
                Amp4Center.getStart().alongWith(Commands.run(() -> intake.setIntakeOutput(IntakeConstants.INTAKE_OUTPUT))).withTimeout(3)
                .finallyDo(() -> intake.stop())
                .andThen(new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led))
                .andThen(Amp4Center.getNext())
                .andThen(new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led))
                .andThen(Amp4Center.getEnd())
                .andThen(new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led)),
            printTimer()
        );
    }

    public static Command Amp6Center(Swerve swerve, Elevator elevator, Pivot pivot, Intake intake, Conveyor conveyor){
        currentSequence = Amp6Center;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp6Center.getStartingPose()),
            Amp6Center.getStart(),
            new WaitCommand(0.5),
            Amp6Center.getNext(),
            new WaitCommand(0.5),
            Amp6Center.getNext(),
            new WaitCommand(0.5),
            Amp6Center.getNext(),
            //new AutoIntakeFloor(swerve, elevator, pivot, intake, conveyor),
            Amp6Center.getNext(),
            new WaitCommand(0.5),
            Amp6Center.getNext(),
            //new AutoIntakeFloor(swerve, elevator, pivot, intake, conveyor),
            Amp6Center.getEnd(),
            printTimer()
        );
    }

    public static Command Source4Center(Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, LED led){
        currentSequence = Source4Center;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Source4Center.getStartingPose()),
            new FacePoint(swerve, new DoubleSupplier[]{() -> 0, () -> 0}, AllianceFlippable.getTargetSpeaker().getTranslation(), false).withTimeout(0.5),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Source4Center.getIndex(0),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Source4Center.getIndex(1),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            //Source4Center.getIndex(2),
            //new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor),
/*             Source4Center.getIndex(3),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor), */
            printTimer()
        );
    }

    public static Command Amp5_1Center(Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Amp5_1Center;
        return new SequentialCommandGroup(
            startTimer(),
            Amp4Close(swerve, shooter, elevator, pivot, conveyor, intake, led),
            Amp5_1Center.getIndex(0),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Amp5_1Center.getIndex(1),
            new AutoIntakeFloor(elevator, pivot, conveyor, intake, led),
            printTimer()
        );
    }

    public static Command Amp5_NoCenter(Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Amp5_1Center;
        return new SequentialCommandGroup(
            startTimer(),
            Amp4Close(swerve, shooter, elevator, pivot, conveyor, intake, led),
            Amp5_1Center.getIndex(0),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer()
        );
    }

    public static Command Amp5Center_2(Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Amp5Center_2;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp5Center_2.getStartingPose()),
            new ShootSequence(SuperstructureState.SUBWOOFER, ShooterConstants.SUBWOOFER_VELOCITY + 500.0, elevator, pivot, shooter, conveyor),
            Amp5Center_2.getIndex(0),
            new AutoIntakeFloor(elevator, pivot, conveyor, intake, led),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Amp5Center_2.getIndex(1),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Amp5Center_2.getIndex(2),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Amp5Center_2.getIndex(3),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer()
        );
    }

    public static Command Source3_NoStage(Swerve swerve, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Source3_NoStage;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Source3_NoStage.getStartingPose()),
            Source3_NoStage.getIndex(0),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Source3_NoStage.getIndex(1),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            Source3_NoStage.getIndex(2),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer()
            /* startTimer(),
            swerve.resetPoseCommand(Source3_NoStage.getStartingPose()),
            Source3_NoStage.getIndex(0),
            new SmartShootSequence(true, true, swerve, elevator, pivot, shooter, conveyor, led),
            Source3_NoStage.getIndex(1),
            new SmartShootSequence(true, true, swerve, elevator, pivot, shooter, conveyor, led),
            Source3_NoStage.getIndex(2),
            new SmartShootSequence(true, true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer() */
        );
    }

    public static Command SweepSource(Swerve swerve, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake){
        currentSequence = SweepCenter;
        return new ParallelCommandGroup(
            swerve.resetPoseCommand(SweepCenter.getStartingPose()),
            pivot.angleCommand(PivotConstants.HOME_ANGLE),
            shooter.dutyCycleCommand(0.2),
            conveyor.dutyCycleCommand(1.0),
            intake.dutyCycleCommand(1.0),
            SweepCenter.getIndex(0)
            .andThen(SweepCenter.getIndex(1))
        );
    }

    public static Command Sweep3_Source(Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Sweep3_Source;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Sweep3_Source.getStartingPose()),
            new ParallelRaceGroup(
                intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT),
                pivot.angleCommand(PivotConstants.HOME_ANGLE),
                elevator.heightCommand(ElevatorConstants.HOME_HEIGHT),
                shooter.dutyCycleCommand(0.2),
                conveyor.dutyCycleCommand(1.0),
                Sweep3_Source.getIndex(0).andThen(Sweep3_Source.getIndex(1))
                .andThen(new WaitCommand(0.5))
            ),
            shooter.stopCommand().alongWith(conveyor.stopCommand()),
            Sweep3_Source.getIndex(2),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer()
        );
    }

    public static Command Sweep2_Source(Swerve swerve, Pivot pivot, Elevator elevator, Shooter shooter, Conveyor conveyor, Intake intake, LED led){
        currentSequence = Sweep2_Source;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Sweep2_Source.getStartingPose()),
            new ParallelRaceGroup(
                intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT),
                pivot.angleCommand(PivotConstants.HOME_ANGLE),
                elevator.heightCommand(ElevatorConstants.HOME_HEIGHT),
                shooter.dutyCycleCommand(0.2),
                conveyor.dutyCycleCommand(1.0),
                Sweep2_Source.getIndex(0)
                .andThen(new WaitCommand(0.5))
            ),
            shooter.stopCommand().alongWith(conveyor.stopCommand()),
            Sweep2_Source.getIndex(1),
            new AutoShootSequence(true, swerve, elevator, pivot, shooter, conveyor, led),
            printTimer()
        );
    }

    public static Command Leave(Swerve swerve, double delay){
        currentSequence = Leave;
        return new SequentialCommandGroup(
            swerve.resetPoseCommand(Leave.getStartingPose()),
            new WaitCommand(delay),
            Leave.getIndex(0)
        );
    }
}
