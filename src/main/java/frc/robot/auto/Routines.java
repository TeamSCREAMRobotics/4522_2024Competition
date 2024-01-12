package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

public class Routines {

    private static PathPlannerPath getPath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    public static Command testAuto(){
        return new PathPlannerAuto("Test");
    }


}
