package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shuffleboard.tabs.MatchTab;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 */
public class Autonomous{

    private static final SendableChooser<Command> m_autoChooser = MatchTab.getAutoChooser();

    public record PPEvent(String name, Command command){}

    private static boolean configured = false;

    public static void configure(Command defaultCommand, PPEvent... ppEvents){
        if(configured){
            DriverStation.reportWarning("Auto already configured!", true);
            return;
        }

        for(PPEvent ppEvent : ppEvents){
            NamedCommands.registerCommand(ppEvent.name(), ppEvent.command());
        }

        m_autoChooser.setDefaultOption(defaultCommand.getName(), defaultCommand);
        configured = true;
    }

    public static Command getSelected(){
        return m_autoChooser.getSelected();
    }

    public static void addRoutines(Command... routines){
        for(Command routine : routines){
            m_autoChooser.addOption(routine.getName(), routine);
        }
    }
}
