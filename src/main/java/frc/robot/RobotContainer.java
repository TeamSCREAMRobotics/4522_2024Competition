package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.Autonomous;
import frc.robot.auto.Autonomous.PPEvent;
import frc.robot.auto.Routines;
import frc.robot.commands.elevator.ElevatorManualCommand;
import frc.robot.commands.elevator.ElevatorTargetCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.pivot.PivotManualCommand;
import frc.robot.commands.pivot.PivotTargetCommand;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.TrackDetectorTarget;
import frc.robot.controlboard.Controlboard;
import frc.robot.shuffleboard.ShuffleboardTabManager;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

    private static final Optional<Alliance> m_alliance = DriverStation.getAlliance();
    
    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();
    //private static final Shooter m_Shooter = new Shooter();
    //private static final Pivot m_pivot = new Pivot();
    //private static final Elevator m_elevator = new Elevator();
    //private static final Conveyor m_conveyor = new Conveyor();
    private static final Intake m_intake = new Intake();


    private static final ShuffleboardTabManager m_shuffleboardTabManager = new ShuffleboardTabManager();
    
    /**
     * Configures the basic robot systems, such as Shuffleboard, autonomous, default commands, and button bindings.
     */
    public RobotContainer() {
        m_shuffleboardTabManager.addTabs(true);
        configButtonBindings();
        configDefaultCommands();
        configAuto();
    }

    /**
     * Configures button bindings from Controlboard.
     */
    private void configButtonBindings() {
        Controlboard.getZeroGyro().onTrue(new InstantCommand(() -> m_swerve.resetGyro(AllianceFlippable.ForwardRotation())));

        /* Conveyor */
        //Controlboard.getAutoFire().toggleOnTrue(new InstantCommand(() -> m_conveyor.setConveyorSpeed(0.75))); //TODO Should be toggle or timer based or etc? Should it have an output based on a joystick or triger?

        /* Elevator */
        //Controlboard.getManualElevator_Boolean().toggleOnTrue(new ElevatorManualCommand(m_elevator, Controlboard.getManualElevator_Output()));

        /* Pivot */
        //Controlboard.getManualPivot_Boolean().toggleOnTrue(new PivotManualCommand(m_pivot, Controlboard.getManualPivot_Output()));

        /* Shooter */
        //Controlboard.getManualFire().whileTrue(new InstantCommand(() -> m_Shooter.setShooterOutput(0.75))); //TODO Should be toggle or timer based or etc? Should it have an output based on a joystick or triger?

        /* Auto Shot */
        //Controlboard.getPrepShot().toggleOnTrue(new ParallelCommandGroup(new PivotTargetCommand(m_pivot, Rotation2d.fromDegrees(PivotConstants.pivotTreeMap.get(null))), new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorTreeMap.get(null)))); //TODO needs to input the distance from the speaker as the key

        Controlboard.getManualIntake().whileTrue(new IntakeCommand(m_intake, IntakeConstants.INTAKE_SPEED)).onFalse(new IntakeCommand(m_intake, 0));
        Controlboard.getEjectIntake().whileTrue(new IntakeCommand(m_intake, IntakeConstants.EJECT_SPEED)).onFalse(new IntakeCommand(m_intake, 0));
    }

    private void configDefaultCommands() { 
        /* Sets the default command for the swerve subsystem */
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve,
                Controlboard.getTranslation(),
                Controlboard.getRotation(),
                Controlboard.getFieldCentric()
            )
        );
    }

    /**
     * Configures auto. 
     * Configure default auto and named commands with configure(Command defaultAuto, NamedCommand... namedCommands)<p>
     *  ^^ THE ABOVE STEP MUST BE DONE FIRST ^^ <p>
     * Add auto routines with addCommands(Command... commands)
     */
    private void configAuto() {
        Autonomous.configure(
            Commands.none().withName("Do Nothing"),
            new PPEvent("ExampleEvent", new PrintCommand("This is an example event :)"))
        );

        Autonomous.addRoutines(
            Routines.testAuto().withName("Test Auto")
        );
    }

    /**
     * Retrieves the selected autonomous command from the autonomous chooser.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        System.out.println("[Auto] Selected auto routine: " + Autonomous.getSelected().getName());
        return Autonomous.getSelected();
    }

    /**
     * Retrieves the Swerve subsystem.
     *
     * @return The Swerve subsystem.
     */
    public static Swerve getSwerve() {
        return m_swerve;
    }
    
    /**
     * Retrieves the current Alliance as detected by the DriverStation. 
     * Use this opposed to DriverStation.getAlliance().
     * @return The current Alliance.
     */
    public static Alliance getAlliance(){
        if(m_alliance.isPresent()){
            return m_alliance.get();
        } else {
            return Alliance.Blue;
        }
    }
}
