package frc.robot.auto;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.FieldConstants;

public class PathCorrectionHandler {
    
    private static ArrayList<Pose2d> list = new ArrayList<Pose2d>();
    static{
        for(Translation2d translation : FieldConstants.CENTER_PIECE_TRANSLATIONS){
            list.add(new Pose2d(translation, AllianceFlippable.ForwardRotation()));
        }
    }

    private static Iterator<Pose2d> iterator = list.iterator();

    public static Pose2d getTargetPose(){
        return new Pose2d();
    }
}
