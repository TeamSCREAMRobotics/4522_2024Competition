package com.team4522.lib.util;

import java.util.ArrayList;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.subsystems.Vision.IntakePipeline;

public class PathSequence {
    ArrayList<PathPlannerPath> list = new ArrayList<PathPlannerPath>();
    String[] pathNames;
    int index = 0;
    Side side;
    
    public PathSequence(Side side, String... pathNames){
        this.pathNames = pathNames;
        this.side = side;
        for(String pathName : pathNames){
            list.add(getPath(pathName));
        }
    }

    public enum Side{
        AMP, CENTER, SOURCE;
    }

    public IntakePipeline getIntakePipeline(){
        return AllianceFlippable.getIntakePipeline(side);
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

    public Command getEnd(){
        return getPathCommand(list.get(list.size()));
    }

    public Side getSide(){
        return side;
    }

    public Command getNext(){
        index ++;
        if(index > list.size()){
            DriverStation.reportWarning("[Auto] No additional paths. Last supplied path: " + pathNames[pathNames.length-1], true);
            return Commands.none();
        }

        return getPathCommand(list.get(index));
    }

    public Command getAll(){
        Command[] commands = new Command[list.size()];
        for(int i = 0; i < list.size(); i++){
            commands[i] = getPathCommand(list.get(i));
        }
        return new SequentialCommandGroup(commands);
    }

    public Command getIndex(int index){
        if(index > list.size()){
            DriverStation.reportWarning("[Auto] No path at index: " + index, true);
            return Commands.none();
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