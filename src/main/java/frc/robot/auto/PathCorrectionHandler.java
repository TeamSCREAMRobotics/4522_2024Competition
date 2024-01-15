package frc.robot.auto;

import java.lang.reflect.Array;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.FieldConstants;

public class PathCorrectionHandler {
    
    private PathPlannerPath[] paths = new PathPlannerPath[5];

    public enum CenterPiece{
        ONE(1), TWO(2), THREE(3), FOUR(4), FIVE(5);

        int index;

        private CenterPiece(int index){
            this.index = index;
        }
    }

    public enum Direction{
        TO_AMP, TO_SOURCE, AUTO;
    }

    private CenterPiece targetPiece;
    private Direction direction;
    private int maxCorrections;

    public PathCorrectionHandler(CenterPiece targetPiece, Direction direction, int maxCorrections){
        this.targetPiece = targetPiece;
        this.direction = direction;
        this.maxCorrections = maxCorrections;
        for(int i = 1; i <= 5; i++){
            paths[i] = PathPlannerPath.fromPathFile("Center" + String.valueOf(i));
        }
    }
}
