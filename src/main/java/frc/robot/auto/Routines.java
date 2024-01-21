package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.AllianceFlippable;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.intake.IntakeManualCommand;
import frc.robot.shuffleboard.tabs.MatchTab;
import frc.robot.subsystems.swerve.Swerve;

public class Routines {

    //private static PathCorrectionHelper correctionHelper;

    private static final Timer autoTimer = new Timer();
    private static final PathSequence Close4 = new PathSequence("Close4_1", "Close4_2", "Close4_3");
    private static final PathSequence AmpSide6 = new PathSequence("Close4_1", "Close4_2", "Amp6_0", "Amp6_1", "Amp6_2", "Amp6_3", "Amp6_4");
    private static final PathSequence SourceSide4 = new PathSequence("SourceSide4_1", "SourceSide4_2", "SourceSide4_3", "SourceSide4_4", "SourceSide4_5", "SourceSide4_6");

    public static Command overrideRotationTarget(Rotation2d rotation){
        return new InstantCommand(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(rotation)));
    }

    public static Command stopOverridingRotationTarget(){
        return new InstantCommand(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
    }

    private static Command resetPoseCommand(Swerve swerve, Pose2d pose){
        return new InstantCommand(() -> swerve.resetPose(pose));
    }

    private static Command startTimerCommand(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> autoTimer.reset()),
            new InstantCommand(() -> autoTimer.start())
        );
    }

    private static Command printTimerCommand(){
        return new InstantCommand(() -> System.out.println("[Auto] Time elapsed:  " + autoTimer.get()));
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
 
    public static Command Close4(Swerve swerve){
        return new SequentialCommandGroup(
            startTimerCommand(),
            resetPoseCommand(swerve, Close4.getStartingPose()),
            Close4.getAll(),
            printTimerCommand()
        );
    }

    public static Command AmpSide6(Swerve swerve){
        return new SequentialCommandGroup(
            startTimerCommand(),
            resetPoseCommand(swerve, AmpSide6.getStartingPose()),
            AmpSide6.getStart(),
            // new WaitCommand(0.75),
            AmpSide6.getNext(),
            // new WaitCommand(0.75),
            AmpSide6.getNext(),
            // new WaitCommand(0.75),
            AmpSide6.getNext(),
            AmpSide6.getNext(),
            // new WaitCommand(0.75),
            AmpSide6.getNext(),
            AmpSide6.getNext(),
            AmpSide6.getNext(),
            printTimerCommand()
        );
    }

    public static Command SourceSide4(Swerve swerve){
        return new SequentialCommandGroup(
            startTimerCommand(),
            resetPoseCommand(swerve, SourceSide4.getStartingPose()),
            SourceSide4.getStart(),
            // new WaitCommand(0.75),
            SourceSide4.getNext(),
            SourceSide4.getNext(),
            // new WaitCommand(0.75),
            SourceSide4.getNext(),
            SourceSide4.getNext(),
            // new WaitCommand(0.75),
            SourceSide4.getNext(),
            printTimerCommand()
        );
    }
}
