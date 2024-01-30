package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.MatchTab;
import frc.robot.shuffleboard.tabs.ShooterTab;
import frc.robot.shuffleboard.tabs.SwerveTab;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Manages tabs for Shuffleboard.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    private static final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

    private Swerve swerve;
    private Shooter shooter;

    public ShuffleboardTabManager(Swerve swerve, Shooter shooter){
        this.swerve = swerve;
        this.shooter = shooter;
    }

    /**
     * Adds predefined tabs to Shuffleboard.<p>
     * @param includeDebug Whether to include additional debug tabs.
     */
    public void addTabs(boolean includeDebug){
        m_tabs.add(new MatchTab(swerve));
        if (includeDebug) {
            m_tabs.add(new SwerveTab(swerve));
            m_tabs.add(new ShooterTab(shooter));
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