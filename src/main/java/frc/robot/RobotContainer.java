package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.auto.Autonomous;
import frc.robot.auto.Autonomous.PPEvent;
import frc.robot.auto.Routines;
import frc.robot.commands.AutoPrepCommand;
import frc.robot.commands.elevator.ElevatorTargetCommand;
import frc.robot.commands.intake.IntakeCommand;
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
    private static final Shooter m_shooter = new Shooter();
    private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    private static final Conveyor m_conveyor = new Conveyor();
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
        Controlboard.getBTestButton().onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(FieldConstants.RED_PODIUM, Rotation2d.fromDegrees(0)))));

        /* Conveyor */
        // Controlboard.getFire().toggleOnTrue(new InstantCommand(() -> m_conveyor.setConveyorSpeed(0.75)));

        /* Elevator */
        // Controlboard.getManualMode().toggleOnTrue(new ElevatorManualCommand(m_elevator, Controlboard.getManualElevator_Output())).onFalse(new ElevatorManualCommand(m_elevator, () -> 0.0));
        // Controlboard.setElevatorPosition_Home().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorSubwooferShotPosition));
        // Controlboard.setElevatorPosition_Subwoofer().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorHomePosition));
        // Controlboard.setElevatorPosition_Amp().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorAmpShotPosition));
        // Controlboard.setElevatorPosition_Trap().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.elevatorTrapShotPosition));

        /* Pivot */
        // Controlboard.getManualMode().toggleOnTrue(new PivotManualCommand(m_pivot, Controlboard.getManualPivot_Output())).onFalse(new PivotManualCommand(m_pivot, () -> 0.0));
        // Controlboard.setPivotPosition_Home().toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.pivotHomeAngle));
        // Controlboard.setPivotPosition_Subwoofer().toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.pivotSubwooferShotAngle));
        // Controlboard.setPivotPosition_Amp().toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.pivotAmpShotAngle));
        // Controlboard.setPivotPosition_Trap().toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.pivotTrapShotAngle));

        /* Shooter */
        // Controlboard.getManualPrepFire().whileTrue(new InstantCommand(() -> m_shooter.setShooterOutput(0.75))).onFalse(new InstantCommand(() -> m_shooter.setShooterOutput(0.0)));

        /* Auto Shot */
        // Controlboard.getAutoPrepShot().toggleOnTrue(new ParallelCommandGroup(new AutoPrepCommand(getAlliance(), m_pivot, m_elevator, m_swerve.getPose()), new FaceSpeakerCommand(m_swerve, getAlliance(), Controlboard.getTranslation(), AllianceFlippable.Translation2d(FieldConstants.BLUE_SPEAKER_OPENING.toTranslation2d(), FieldConstants.RED_SPEAKER_OPENING.toTranslation2d()))));

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
