package frc.robot.auto;

import java.util.ArrayList;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.tabs.MatchTab;

public class PathCorrectionHelper extends SubsystemBase{

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

    private PathPlannerPath[] paths = new PathPlannerPath[5];
    private ArrayList<Command> commands = new ArrayList<Command>();
    private CenterPiece targetPiece;
    private Direction direction;
    private int maxCorrections;
    private int currentIndex;

    public PathCorrectionHelper(){
        commands = new ArrayList<Command>();
        currentIndex = targetPiece.index;
    }

    public Command getCommand(int index){
        return new Command() {};
    }

    @Override
    public void periodic() {
        maxCorrections = MatchTab.getSelectedMaxCorrections();
        targetPiece = MatchTab.getSelectedCenterPiece();
        direction = MatchTab.getSelectedDirection();
    }
}
