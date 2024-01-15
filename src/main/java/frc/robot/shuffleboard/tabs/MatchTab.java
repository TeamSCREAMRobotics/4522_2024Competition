
package frc.robot.shuffleboard.tabs;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.shuffleboard.ShuffleboardTabBase;
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

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Match");

        m_autoChooserEntry = createSendableEntry("Auto Chooser", m_autoChooser, new EntryProperties(0, 0, 2, 1));
        m_field = createSendableEntry("Field", m_field2d, new EntryProperties(2, 0));
    }

    @Override
    public void periodic() {
        m_field2d.setRobotPose(swerve.getPose());
    }

    public static SendableChooser<Command> getAutoChooser(){
        return m_autoChooser;
    }
}
