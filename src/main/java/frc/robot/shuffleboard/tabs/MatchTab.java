
package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.PathCorrectionHelper.CenterPiece;
import frc.robot.auto.PathCorrectionHelper.Direction;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.swerve.Swerve;

public class MatchTab extends ShuffleboardTabBase {
    
    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    private static SendableChooser<CenterPiece> m_centerPieceChooser = new SendableChooser<CenterPiece>();
    private static SendableChooser<Direction> m_directionChooser = new SendableChooser<Direction>();
    private Swerve swerve;

    public MatchTab(Swerve swerve) {
        this.swerve = swerve;
    }

    private ComplexWidget m_autoChooserEntry;
    private ComplexWidget m_centerPieceChooserEntry;
    private ComplexWidget m_directionChooserEntry;

    private ComplexWidget m_field;
    private Field2d m_field2d = new Field2d();

    private static GenericEntry m_maxCorrectionsEntry;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Match");

        m_centerPieceChooser.setDefaultOption("FIVE", CenterPiece.FIVE);
        m_directionChooser.setDefaultOption("TO_AMP", Direction.TO_AMP);

        m_autoChooserEntry = createSendableEntry("Auto Chooser", m_autoChooser, new EntryProperties(0, 0, 2, 1));
        m_centerPieceChooserEntry = createSendableEntry("Center Piece Chooser", m_centerPieceChooser, new EntryProperties(0, 1));
        m_directionChooserEntry = createSendableEntry("Direction Chooser", m_directionChooser, new EntryProperties(1, 1));

        m_maxCorrectionsEntry = createNumberEntry("Max Corrections", 0, new EntryProperties(0, 2));

        m_field = createSendableEntry("Field", m_field2d, new EntryProperties(2, 0, 6, 4));
    }

    @Override
    public void periodic() {
        m_field2d.setRobotPose(swerve.getPose());
    }

    public static SendableChooser<Command> getAutoChooser(){
        return m_autoChooser;
    }

    public static CenterPiece getSelectedCenterPiece(){
        return m_centerPieceChooser.getSelected();
    }

    public static Direction getSelectedDirection(){
        return m_directionChooser.getSelected();
    }

    public static int getSelectedMaxCorrections(){
        return (int) m_maxCorrectionsEntry.getDouble(0);
    }
}
