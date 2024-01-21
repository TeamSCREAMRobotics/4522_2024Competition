package frc.robot;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.Optional;

import javax.swing.text.html.Option;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.Autonomous;
import frc.robot.auto.Autonomous.PPEvent;
import frc.robot.auto.Routines;
import frc.robot.commands.AutoPrepCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.climber.AutoClimbCommand;
import frc.robot.commands.conveyor.ConveyorManualCommand;
import frc.robot.commands.elevator.ElevatorManualCommand;
import frc.robot.commands.elevator.ElevatorTargetCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.pivot.PivotManualCommand;
import frc.robot.commands.pivot.PivotTargetCommand;
import frc.robot.commands.shooter.ShooterManualCommand;
import frc.robot.commands.swerve.FacePointCommand;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.FaceGamePieceCommand;
import frc.robot.controlboard.Controlboard;
import frc.robot.shuffleboard.ShuffleboardTabManager;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

    private static Optional<Alliance> m_alliance = DriverStation.getAlliance();
    
    /* Subsystems */
    private static Swerve m_swerve = new Swerve();
/*     private static final Climber m_climber = new Climber();
    private static final Shooter m_shooter = new Shooter();
    private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    private static final Conveyor m_conveyor = new Conveyor(); */
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
        Controlboard.getZeroGyro().onTrue(new InstantCommand(() -> m_swerve.resetGyro(AllianceFlippable.getForwardRotation())));
        Controlboard.getResetPose().onTrue(new InstantCommand(() -> m_swerve.resetPose(new Pose2d(FieldConstants.RED_PODIUM, AllianceFlippable.getForwardRotation()))));
        //Controlboard.getBTestButton().whileTrue(new FeedForwardCharacterization(m_elevator, true, new FeedForwardCharacterizationData("Elevator"), m_elevator::setElevatorVoltage, m_elevator::getElevatorVelocity, m_elevator::getElevatorAcceleration));

        /* Conveyor */
        // Controlboard.getFire_Speaker().toggleOnTrue(new ConveyorManualCommand(m_conveyor, ConveyorConstants.SPEAKER_SPEED));

        /* Elevator */
        // Controlboard.getManualMode().toggleOnTrue(new ElevatorManualCommand(m_elevator, Controlboard.getManualElevator_Output()));

        /* Pivot */
        // Controlboard.getManualMode().toggleOnTrue(new PivotManualCommand(m_pivot, Controlboard.getManualPivot_Output()));

        /* Pivot AND Elevator */
        // Controlboard.setPosition_Home().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_HOME_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_HOME_ANGLE));
        // Controlboard.setPosition_Subwoofer().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_SUBWOOFER_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_SUBWOOFER_ANGLE));
        // Controlboard.setPosition_Amp().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_AMP_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_AMP_ANGLE));
        // Controlboard.setPosition_Trap().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_TRAP_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_TRAP_ANGLE));

        /* Shooter */
        // Controlboard.getManualShooter().toggleOnTrue(new ShooterManualCommand(m_shooter, ShooterConstants.SHOOTER_SHOOT_OUTPUT));
        // Controlboard.getEjectShooter().toggleOnTrue(new ShooterManualCommand(m_shooter, ShooterConstants.SHOOTER_EJECT_OUTPUT));

        /* Automation */
        // Controlboard.getAutoPrepShot().toggleOnTrue(new AutoPrepCommand(m_pivot, m_elevator, m_shooter, m_swerve, Controlboard.getDefense().getAsBoolean(), getAlliance())).toggleOnTrue(new FacePointCommand(m_swerve, getAlliance(), Controlboard.getTranslation(), AllianceFlippable.Translation2d(FieldConstants.BLUE_SPEAKER_OPENING, FieldConstants.RED_SPEAKER_OPENING)));
        /*Controlboard.getAutoclimb().toggleOnTrue(
            new SequentialCommandGroup(
            new FacePointCommand(m_swerve, getAlliance(), Controlboard.getTranslation(), AllianceFlippable.Translation2d(FieldConstants.BLUE_STAGE_RIGHT, FieldConstants.RED_STAGE_RIGHT)), //how to select which stage we are going to, if we just faced the direct center of the stage would it function the same?
                // new AutoAllignCommand(), //TODO
                new AutoClimbCommand(m_climber), //Will we be able to climb and move the elevator/pivot at the same time/
                new ParallelCommandGroup(
                    new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_TRAP_POSITION),
                    new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_TRAP_ANGLE)
                ),
                new ConveyorManualCommand(m_conveyor, ConveyorConstants.AMP_TRAP_SPEED)
            )
        );*/

        /* Intake */
        //Controlboard.getManualIntake().whileTrue(new IntakeManualCommand(m_intake, IntakeConstants.INTAKE_SPEED, true));
        //Controlboard.getEjectIntake().whileTrue(new IntakeManualCommand(m_intake, IntakeConstants.EJECT_SPEED, true));
        //Controlboard.getAutoPickup().whileTrue(new FaceGamePieceCommand(m_swerve, Controlboard.getTranslation(), SwerveConstants.VISION_ROTATION_CONSTANTS));
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

        /*m_climber.setDefaultCommand(
            new ClimberManualCommand(
                m_climber,
                0.0
            )
        );*/
        
        /*m_conveyor.setDefaultCommand(
            new ConveyorManualCommand(
                m_conveyor, 
                0.0)
        );
        
        m_elevator.setDefaultCommand(
            new ElevatorTargetCommand(
                m_elevator, 
                ElevatorConstants.ELEVATOR_HOME_POSITION)
        );*/

        /* m_intake.setDefaultCommand(
            new IntakeManualCommand(
                m_intake, 
                0.0)
        );

        m_pivot.setDefaultCommand(
            new PivotTargetCommand(
                m_pivot, 
                PivotConstants.PIVOT_HOME_ANGLE)
        );
        
        m_shooter.setDefaultCommand(
            new ShooterManualCommand(
                m_shooter, 
                0.0)
        );*/
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
            new PPEvent("StartIntake", new InstantCommand()),//new IntakeAutoCommand(m_intake, IntakeConstants.INTAKE_SPEED, false)),
            new PPEvent("StopIntake", new InstantCommand()),
            new PPEvent("StartAimAtSpeaker", Routines.overrideRotationTarget(FacePointCommand.calculateAngleToPoint(m_swerve.getPose().getTranslation(), AllianceFlippable.getTargetSpeaker()))),
            new PPEvent("StopAimAtSpeaker", Routines.stopOverridingRotationTarget())
        );

        Autonomous.addRoutines(
            Routines.AmpSide6(m_swerve).withName("AmpSide6")
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

    /* public static Intake getIntake(){
        return m_intake;
    } */

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

    public static void stopAll(){
        //m_shooter.stop();
        //m_pivot.stop();
        //m_elevator.stop();
        //m_conveyor.stop();
        //m_intake.stop();
        m_swerve.stopAll();
    }
    
    public static void setAllNeutralModes(NeutralModeValue mode){
        //m_shooter.setNeutralMode(mode);
        //m_pivot.setNeutralMode(mode);
        //m_elevator.setNeutralMode(mode);
        //m_conveyor.setNeutralMode(mode);
        //m_intake.setNeutralMode(mode);
        m_swerve.setNeutralModes(mode, mode);
    }
}