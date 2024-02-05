package frc2024.dashboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.RobotContainer;
import frc2024.dashboard.tabs.MatchTab;
import frc2024.dashboard.tabs.ShooterTab;
import frc2024.dashboard.tabs.SwerveTab;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

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
            if(swerve != null){
                m_tabs.add(new SwerveTab(swerve));
            }
            
            if(shooter != null){
                m_tabs.add(new ShooterTab(shooter));
            }
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