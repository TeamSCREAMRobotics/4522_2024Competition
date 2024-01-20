package frc.robot.auto;

import java.util.ArrayList;
import java.util.Iterator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AllianceFlippable;
import frc.robot.subsystems.swerve.Swerve;

public class PathSequence {
    ArrayList<PathPlannerPath> list = new ArrayList<PathPlannerPath>();
    String[] pathNames;
    int index = 0;
    
    public PathSequence(String... pathNames){
        this.pathNames = pathNames;
        for(String pathName : pathNames){
            list.add(getPath(pathName));
        }
    }

    private static PathPlannerPath getPath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    private static Command getPathCommand(PathPlannerPath path){
        return AutoBuilder.followPath(path);
    }

    private static Pose2d getPathStartingPose(PathPlannerPath path){
        return AllianceFlippable.MirroredPose2d(path.getPreviewStartingHolonomicPose());
    }

    public Pose2d getStartingPose(){
        return getPathStartingPose(list.get(0));
    }

    public Command getStart(){
        return getPathCommand(list.get(0));
    }

    public Command getNext(){
        index ++;
        if(index > list.size()-1){
            DriverStation.reportWarning("[Auto] No additional paths | Last supplied path: " + pathNames[pathNames.length-1], true);
            return new InstantCommand();
        }

        return getPathCommand(list.get(index));
    }

    public Command getAll(){
        Command[] commands = new Command[0];
        commands = list.toArray(commands);
        return new SequentialCommandGroup(commands);
    }

    public Command getIndex(int index){
        if(index > list.size()-1){
            DriverStation.reportWarning("[Auto] No path at specified index", true);
            return new InstantCommand();
        }
        return getPathCommand(list.get(index));
    }

    public PathSequence withAdditional(String... pathNames){
        for(String pathName : pathNames){
            list.add(getPath(pathName));
        }
        return this;
    }
}
