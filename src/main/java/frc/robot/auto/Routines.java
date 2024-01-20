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

    private static Timer autoTimer = new Timer();
    private static Timer rotationOverrideTimer = new Timer();

    private static PathPlannerPath getPath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    private static Command getPathCommand(String pathName){
        return AutoBuilder.followPath(getPath(pathName));
    }

    private static Pose2d getPathStartingPose(String pathName){
        return AllianceFlippable.MirroredPose2d(getPath(pathName).getPreviewStartingHolonomicPose());
    }

    private static Command resetPoseCommand(Swerve swerve, String pathName){
        return new InstantCommand(() -> swerve.resetPose(getPathStartingPose(pathName)));
    }

    private static void overrideRotationTarget(Rotation2d rotation, double seconds){
        rotationOverrideTimer.start();
        Optional<Rotation2d> target;
        if(rotationOverrideTimer.get() <= seconds){
            target = Optional.of(rotation);
        } else{
            target = Optional.empty();
            rotationOverrideTimer.reset();
        }
        
        PPHolonomicDriveController.setRotationTargetOverride(() -> target);
    }

    private static Command resetTimerCommand(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> autoTimer.reset()),
            new InstantCommand(() -> autoTimer.start())
        );
    }

    private static Command printTimerCommand(){
        return new InstantCommand(() -> System.out.println("[Auto] Time taken:  " + autoTimer.get()));
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
            resetPoseCommand(swerve, "Close4_1"),
            getPathCommand("Close4_1"),
            new WaitCommand(1),
            getPathCommand("Close4_2"),
            new WaitCommand(1),
            getPathCommand("Close4_3")
        );
    }

    public static Command Close5(Swerve swerve){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                resetTimerCommand(),
                resetPoseCommand(swerve, "Close4_1")
                // new WaitCommand(0.5)
            ),
            getPathCommand("Close4_1"),
            // new WaitCommand(0.75),
            getPathCommand("Close4_2"),
            // new WaitCommand(0.75),
            getPathCommand("Close4_3"),
            // new WaitCommand(0.75),
            getPathCommand("Close6_1"),
            getPathCommand("Close6_2"),
            // new WaitCommand(0.75),
            getPathCommand("Close6_3"),
            getPathCommand("Close6_4"),
            printTimerCommand()
        );
    }
}
