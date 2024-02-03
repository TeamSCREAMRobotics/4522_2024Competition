
package frc.robot.dashboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.dashboard.ShuffleboardTabBase;
import frc.robot.subsystems.swerve.Swerve;

public class MatchTab extends ShuffleboardTabBase {
    
    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    private Swerve swerve;

    public MatchTab(Swerve swerve) {
        this.swerve = swerve;
    }

    private ComplexWidget m_autoChooserEntry;
    private ComplexWidget m_field;
    private Field2d m_field2d = new Field2d();
    private GenericEntry m_matchTime;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Match");

        m_autoChooserEntry = createSendableEntry("Auto Chooser", m_autoChooser, new EntryProperties(0, 0, 4, 2));
        m_field = createSendableEntry("Field", m_field2d, new EntryProperties(7, 0, 17, 10));
        m_matchTime = createNumberEntry("Match Time", 0, new EntryProperties(0, 2, 8, 4));
    }

    @Override
    public void periodic() {
        m_field2d.setRobotPose(swerve.getPose());

        m_matchTime.setDouble(DriverStation.getMatchTime());
    }

    public static SendableChooser<Command> getAutoChooser(){
        return m_autoChooser;
    }
}
