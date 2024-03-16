package frc2024.dashboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.RobotContainer;
import frc2024.dashboard.tabs.MatchTab;
import frc2024.dashboard.tabs.SubsystemTestTab;
import frc2024.dashboard.tabs.SwerveTab;
import frc2024.subsystems.Stabilizers;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

/**
 * Manages tabs for Shuffleboard.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    private static final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

    private Swerve swerve;
    private Stabilizers stabilizers;
    private Conveyor conveyor;
    private Elevator elevator;
    private Intake intake;
    private Pivot pivot;
    private Shooter shooter;

    public ShuffleboardTabManager(Swerve swerve, Stabilizers stabilizers, Conveyor conveyor, Elevator elevator, Intake intake, Pivot pivot, Shooter shooter){
        this.swerve = swerve;
        this.stabilizers = stabilizers;
        this.conveyor = conveyor;
        this.elevator = elevator;
        this.intake = intake;
        this.pivot = pivot;
        this.shooter = shooter;
    }

    /**
     * Adds predefined tabs to Shuffleboard.<p>
     * @param includeDebug Whether to include additional debug tabs.
     */
    public void addTabs(boolean includeDebug){
        m_tabs.add(new MatchTab(swerve, elevator, pivot));
        if (includeDebug) {
            if(swerve != null){
                m_tabs.add(new SwerveTab(swerve));
            }
                m_tabs.add(new SubsystemTestTab(stabilizers, conveyor, elevator, intake, pivot, shooter));
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
}