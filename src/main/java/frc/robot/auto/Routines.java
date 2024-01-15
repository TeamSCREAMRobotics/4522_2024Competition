package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.AllianceFlippable;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.swerve.Swerve;

public class Routines {

    private static PathPlannerPath getPath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    private static Pose2d getPathStartingPose(String pathName){
        return AllianceFlippable.Pose2d(getPath(pathName).getPreviewStartingHolonomicPose());
    }

    public static Command testAuto(Swerve swerve){
        // Path to beginning piece + replanning check
        // Vision targeting + replanning check
        // Back to speaker
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetPose(getPathStartingPose("Close4_1"))),
            new SequentialCommandGroup(
                AutoBuilder.pathfindThenFollowPath(getPath("Center4"), SwerveConstants.PATH_CONSTRAINTS)
            )
        );
    }
}
