package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.MatchTab;
import frc.robot.shuffleboard.tabs.SwerveTab;

/**
 * Manages tabs for Shuffleboard.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    private static final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

    public ShuffleboardTabManager(){}

    /**
     * Adds predefined tabs to Shuffleboard.<p>
     * @param includeDebug Whether to include additional debug tabs.
     */
    public void addTabs(boolean includeDebug){
        m_tabs.add(new MatchTab());
        if (includeDebug) {
            m_tabs.add(new SwerveTab(RobotContainer.getSwerve()));
        }

        for (ShuffleboardTabBase tab : m_tabs) {
            tab.createEntries();
        }
    }

    /**
     * Calls the periodic method for each Shuffleboard tab in the list of tabs.
     */
    public void periodic() {
        for (ShuffleboardTabBase tab : m_tabs) {
            tab.periodic();
        }
    }

    @Override
    public void register() {
        super.register();
    }
}