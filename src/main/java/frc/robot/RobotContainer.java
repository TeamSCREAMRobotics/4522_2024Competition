package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.auto.Autonomous;
import frc.robot.auto.Autonomous.PPEvent;
import frc.robot.auto.Routines;
import frc.robot.commands.AutoPrepCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.conveyor.ConveyorManualCommand;
import frc.robot.commands.elevator.ElevatorManualCommand;
import frc.robot.commands.elevator.ElevatorTargetCommand;
import frc.robot.commands.intake.IntakeManualCommand;
import frc.robot.commands.pivot.PivotManualCommand;
import frc.robot.commands.pivot.PivotTargetCommand;
import frc.robot.commands.shooter.ShooterManualCommand;
import frc.robot.commands.swerve.FaceSpeakerCommand;
import frc.robot.commands.swerve.TeleopSwerve;
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
    //private static final Shooter m_shooter = new Shooter();
    //private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    //private static final Conveyor m_conveyor = new Conveyor();
    private static final Intake m_intake = new Intake();

    private static final ShuffleboardTabManager m_shuffleboardTabManager = new ShuffleboardTabManager(m_swerve);
    
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
        Controlboard.getBTestButton().whileTrue(new FeedForwardCharacterization(m_elevator, true, new FeedForwardCharacterizationData("Elevator"), m_elevator::setElevatorVoltage, m_elevator::getElevatorVelocity, m_elevator::getElevatorAcceleration));

        /* Conveyor */
        // Controlboard.getFire_Speaker().toggleOnTrue(new ConveyorManualCommand(m_conveyor, ConveyorConstants.SPEAKER_SPEED));

        /* Elevator */
        // Controlboard.getManualMode().toggleOnTrue(new ElevatorManualCommand(m_elevator, Controlboard.getManualElevator_Output()));

        /* Pivot */
        // Controlboard.getManualMode().toggleOnTrue(new PivotManualCommand(m_pivot, Controlboard.getManualPivot_Output()));

        /* Pivot AND Elevator */
        // Controlboard.setPosition_Home().toggleOnTrue(new ParallelCommandGroup(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorHome_Position), new PivotTargetCommand(m_pivot, PivotConstants.pivotHome_Angle)));
        // Controlboard.setPosition_Subwoofer().toggleOnTrue(new ParallelCommandGroup(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorSubwooferShot_Position), new PivotTargetCommand(m_pivot, PivotConstants.pivotSubwooferShot_Angle)));
        // Controlboard.setPosition_Amp().toggleOnTrue(new ParallelCommandGroup(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorAmpShot_Position), new PivotTargetCommand(m_pivot, PivotConstants.pivotAmpShot_Angle)));
        // Controlboard.setPosition_Trap().toggleOnTrue(new ParallelCommandGroup(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorTrapShot_Position), new PivotTargetCommand(m_pivot, PivotConstants.pivotTrapShot_Angle)));

        /* Shooter */
        // Controlboard.getManualShooter().toggleOnTrue(new ShooterManualCommand(m_shooter, ShooterConstants.SHOOTER_SHOOT_SPEED));
        // Controlboard.getEjectShooter().toggleOnTrue(new ShooterManualCommand(m_shooter, ShooterConstants.SHOOTER_EJECT_SPEED));

        /* Auto Shot */
        //Controlboard.getAutoPrepShot().toggleOnTrue(new AutoPrepCommand(m_pivot, m_elevator, m_swerve, getAlliance())).toggleOnTrue(new FaceSpeakerCommand(m_swerve, getAlliance(), Controlboard.getTranslation(), AllianceFlippable.Translation2d(FieldConstants.BLUE_SPEAKER_OPENING, FieldConstants.RED_SPEAKER_OPENING)));

        /* Intake */
        Controlboard.getManualIntake().whileTrue(new IntakeManualCommand(m_intake, IntakeConstants.INTAKE_SPEED));
        Controlboard.getEjectIntake().whileTrue(new IntakeManualCommand(m_intake, IntakeConstants.EJECT_SPEED));
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

        m_intake.setDefaultCommand(
            new IntakeManualCommand(
                m_intake, 
                0)
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
            new PPEvent("ExampleEvent", new PrintCommand("This is an example event :)")),
            new PPEvent("StartIntake", new IntakeManualCommand(m_intake, IntakeConstants.INTAKE_SPEED)),
            new PPEvent("StopIntake", new IntakeManualCommand(m_intake, 0))
        );

        Autonomous.addRoutines(
            Routines.Close5(m_swerve).withName("Close5")
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

     /**
     * Retrieves the Swerve subsystem.
     *
     * @return The Swerve subsystem.
     */
    public static Swerve getSwerve() {
        return m_swerve;
    }

    /* public static Shooter getShooter(){
        return m_shooter;
    }

    public static Pivot getPivot(){
        return m_pivot;
    }

    public static Elevator getElevator(){
        return m_elevator;
    }

    public static Conveyor getConveyor(){
        return m_conveyor;
    } */

    public static Intake getIntake(){
        return m_intake;
    }
}
