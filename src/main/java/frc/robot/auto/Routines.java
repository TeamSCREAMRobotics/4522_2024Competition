package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;

public class Routines {

    /** 
     * Returns a full auto created in the PathPlanner application.
     * Uses Named Commands to trigger commands along the path.
     * @return A full autonomous command.
     */
    public static Command exampleAuto(){
        return new PathPlannerAuto("ExampleAuto");
    }
}
