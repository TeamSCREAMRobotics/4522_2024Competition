
package frc.robot.shuffleboard.tabs;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shuffleboard.ShuffleboardTabBase;

public class MatchTab extends ShuffleboardTabBase {

    public MatchTab() {}
    
    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    private ComplexWidget m_autoChooserEntry;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Match");

        m_autoChooserEntry = createSendableEntry("Auto Chooser", m_autoChooser, new EntryProperties(0, 0, 2, 1));
    }

    @Override
    public void periodic() {}

    public static SendableChooser<Command> getAutoChooser(){
        return m_autoChooser;
    }
}
