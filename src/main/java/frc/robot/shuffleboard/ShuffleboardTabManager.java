package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.MatchTab;
import frc.robot.shuffleboard.tabs.SwerveTab;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Manages tabs for Shuffleboard.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    private static final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

    private Swerve swerve;

    public ShuffleboardTabManager(Swerve swerve){
        this.swerve = swerve;
    }

    /**
     * Adds predefined tabs to Shuffleboard.<p>
     * @param includeDebug Whether to include additional debug tabs.
     */
    public void addTabs(boolean includeDebug){
        m_tabs.add(new MatchTab(swerve));
        if (includeDebug) {
            m_tabs.add(new SwerveTab(swerve));
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