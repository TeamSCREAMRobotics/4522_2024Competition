package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.AllianceFlippable;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class Routines {

    private static PathPlannerPath getPath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    private static Pose2d getPathStartingPose(String pathName){
        return getPath(pathName).getPreviewStartingHolonomicPose();
    }

    public static Command testAuto(){
        // Path to beginning piece + replanning check
        // Vision targeting + replanning check
        // Back to speaker
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.getSwerve().resetPose(getPathStartingPose("TestRed"))),
            new ParallelCommandGroup(
                AutoBuilder.followPath(getPath("TestRed")),
                new WaitCommand(15)
            )
        );
    }
}
