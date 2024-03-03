
package frc2024.dashboard.tabs;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2024.RobotContainer;
import frc2024.Constants.FieldConstants;
import frc2024.auto.Routines;
import frc2024.dashboard.ShuffleboardTabBase;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.swerve.Swerve;

public class MatchTab extends ShuffleboardTabBase {
    
    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    private Swerve swerve;
    private Elevator elevator;
    private Pivot pivot;

    public MatchTab(Swerve swerve, Elevator elevator, Pivot pivot) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;

        checkNulls(this.swerve, this.elevator, this.pivot);
    }

    private ComplexWidget m_autoChooserEntry;
    private ComplexWidget m_field;
    private Field2d m_field2d = new Field2d();
    private GenericEntry m_matchTime;
    
    private static GenericEntry m_elevatorCoast;
    private static GenericEntry m_pivotCoast;

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Match");

        m_autoChooserEntry = createSendableEntry("Auto Chooser", m_autoChooser, new EntryProperties(0, 0, 4, 2));
        m_field = createSendableEntry("Auto Start Position", m_field2d, new EntryProperties(7, 0, 17, 10));
        m_matchTime = createNumberEntry("Match Time", 0, new EntryProperties(0, 2, 8, 4));
        
        m_elevatorCoast = createBooleanEntry("Elevator Coast", false, new EntryProperties(2, 1, 2, 1));
        m_pivotCoast = createBooleanEntry("Pivot Coast", false, new EntryProperties(2, 2, 2, 1));
    }

    @Override
    public void periodic() {
        /* Neutral Modes */
        //pivot.setNeutralMode(m_pivotCoast.getBoolean(false) ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        //elevator.setNeutralMode(m_elevatorCoast.getBoolean(false) ? NeutralModeValue.Coast : NeutralModeValue.Brake);

        //m_field2d.setRobotPose(AllianceFlippable.MirrorPoseForField2d(Routines.getSequenceFromAutoName(m_autoChooser.getSelected().getName()).getStartingPose()));

        m_matchTime.setDouble(DriverStation.getMatchTime());
    }

    public static SendableChooser<Command> getAutoChooser(){
        return m_autoChooser;
    }

    public void checkNulls(Swerve swerve, Elevator elevator, Pivot pivot){
        if(swerve == null){
            swerve = new Swerve();
            DriverStation.reportError("[Dashboard] Swerve not present, instantiating new", true);
        }
        if(elevator == null){
            swerve = new Swerve();
            DriverStation.reportError("[Dashboard] Elevator not present, instantiating new", true);
        }
        if(pivot == null){
            swerve = new Swerve();
            DriverStation.reportError("[Dashboard] Pivot not present, instantiating new", true);
        }
    }
}
