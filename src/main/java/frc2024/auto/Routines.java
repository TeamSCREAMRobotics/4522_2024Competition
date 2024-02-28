package frc2024.auto;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.PathSequence;
import com.team4522.lib.util.PathSequence.Side;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
import frc2024.commands.SuperstructureToPosition;
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
    private static final PathSequence Amp4Close = new PathSequence(Side.AMP, "Amp4Close#1", "Amp4Close#2", "Amp4Close#3");
    private static final PathSequence Amp5Center = new PathSequence(Side.AMP, "Amp4Close#1", "Amp4Close#2", "Amp6Center#0", "Amp6Center#1", "Amp6Center#2");
    private static final PathSequence Amp6Center = new PathSequence(Side.AMP, "Amp4Close#1", "Amp4Close#1", "Amp6Center#0", "Amp6Center#1", "Amp6Center#2", "Amp6Center#3", "Amp6Center#4");
    private static final PathSequence Source4Center = new PathSequence(Side.SOURCE, "Source4Center#1", "Source4Center#2", "Source4Center#3", "Source4Center#4", "Source4Center#5", "Source4Center#6");
    private static final PathSequence Amp4Center = new PathSequence(Side.AMP, "Amp4Center#1", "Amp4Center#2", "Amp4Center#3");
    private static final PathSequence SweepCenter = new PathSequence(Side.SOURCE, "SweepCenter");
    private static final PathSequence Amp5Center_2 = new PathSequence(Side.AMP, "Amp5Center#1", "Amp5Center#2", "Amp5Center#3", "Amp5Center#4");

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

    public static PathSequence getSequenceFromAutoName(String autoName){
        switch (autoName) {
            case "Amp4Close":
            return Amp4Close;
            case "Amp5Center":
            return Amp5Center;
            case "Amp6Center":
            return Amp6Center;
            case "Source4Center":
            return Source4Center;
            case "Amp4Center":
            return Amp4Center;
            case "SweepCenter":
            return SweepCenter;
            case "Amp5Center_2":
            return Amp5Center_2;
            default:
            return new PathSequence(Side.CENTER);
        }
    }

    public static Command testAuto(Swerve swerve){
        PathPlannerPath path = PathPlannerPath.fromPathFile("Test");
        return new SequentialCommandGroup(
            swerve.resetPoseCommand(AllianceFlippable.MirroredPose2d(path.getPreviewStartingHolonomicPose())),
            AutoBuilder.followPath(path)
        );
    }
 
    public static Command Amp4Close(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake){
        currentSequence = Amp4Close;

        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp4Close.getStartingPose()),
            //new ShootSequence(swerve, pivot, shooter, elevator, conveyor)
                Amp4Close.getStart()
                .andThen(new ShootSequence(swerve, pivot, shooter, elevator, conveyor))
                .andThen(Amp4Close.getNext())
                .andThen(new ShootSequence(swerve, pivot, shooter, elevator, conveyor))
                .andThen(Amp4Close.getEnd())
                .andThen(new ShootSequence(swerve, pivot, shooter, elevator, conveyor)),
            printTimer()
        );
    }

    public static Command Amp4Center(Swerve swerve, Shooter shooter, Elevator elevator, Pivot pivot, Conveyor conveyor, Intake intake){
        currentSequence = Amp4Center;
        
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp4Center.getStartingPose()),
            //new ShootSequence(swerve, pivot, shooter, elevator, conveyor))
                Amp4Center.getStart().alongWith(Commands.run(() -> intake.setIntakeOutput(IntakeConstants.INTAKE_OUTPUT))).withTimeout(3)
                .finallyDo(() -> intake.stop())
                .andThen(new ShootSequence(swerve, pivot, shooter, elevator, conveyor))
                .andThen(Amp4Center.getNext())
                .andThen(new ShootSequence(swerve, pivot, shooter, elevator, conveyor))
                .andThen(Amp4Center.getEnd())
                .andThen(new ShootSequence(swerve, pivot, shooter, elevator, conveyor)),
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

    public static Command Source4Center(Swerve swerve){
        currentSequence = Source4Center;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Source4Center.getStartingPose()),
            Source4Center.getStart(),
            // new WaitCommand(0.75),
            Source4Center.getNext(),
            Source4Center.getNext(),
            // new WaitCommand(0.75),
            Source4Center.getNext(),
            Source4Center.getNext(),
            // new WaitCommand(0.75),
            Source4Center.getEnd(),
            printTimer()
        );
    }

    public static Command Amp5Center(Swerve swerve){
        currentSequence = Amp5Center;
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
        Amp6Center.getEnd(),
        new WaitCommand(0.5),
        printTimer()
        );
    }

    public static Command Amp5Center_2(Swerve swerve){
        currentSequence = Amp5Center_2;
        return new SequentialCommandGroup(
            startTimer(),
            swerve.resetPoseCommand(Amp5Center_2.getStartingPose()),
            Amp5Center_2.getStart(),
            Amp5Center_2.getNext(),
            Amp5Center_2.getNext(),
            Amp5Center_2.getEnd(),
            printTimer()
        );
    }

    public static Command SweepCenter(Swerve swerve, Pivot pivot, Shooter shooter, Conveyor conveyor, Intake intake){
        currentSequence = SweepCenter;
        return new ParallelCommandGroup(
            swerve.resetPoseCommand(SweepCenter.getStartingPose()),
            pivot.angleCommand(PivotConstants.HOME_ANGLE),
            shooter.dutyCycleCommand(0.2),
            conveyor.dutyCycleCommand(1.0),
            intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT),
            SweepCenter.getAll()
        );
    }
}
