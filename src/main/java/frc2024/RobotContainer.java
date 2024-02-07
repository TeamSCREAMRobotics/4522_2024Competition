package frc2024;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.FieldConstants;
import frc2024.auto.Autonomous;
import frc2024.auto.Autonomous.PPEvent;
import frc2024.auto.Routines;
import frc2024.commands.climber.ClimberManualCommand;
import frc2024.commands.conveyor.ConveyorCommand;
import frc2024.commands.elevator.ElevatorManualCommand;
import frc2024.commands.elevator.ElevatorPositionCommand;
 import frc2024.commands.pivot.PivotManualCommand;
import frc2024.commands.pivot.PivotPositionCommand;
import frc2024.commands.shooter.ShooterManualCommand;
import frc2024.commands.shooter.ShooterVelocityCommand;
import frc2024.commands.swerve.TeleopDriveCommand;
import frc2024.commands.swerve.DriveToPoseCommand;
import frc2024.commands.swerve.FacePointCommand;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.controlboard.Controlboard;
import frc2024.dashboard.ShuffleboardTabManager;
import frc2024.subsystems.Climber;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.swerve.Swerve;

public class RobotContainer {

    private static Optional<Alliance> m_alliance = DriverStation.getAlliance();
    
    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();
    private static final Climber m_climber = new Climber();
    //private static final Shooter m_shooter = new Shooter();
    //private static final Pivot m_pivot = new Pivot();
    //private static final Elevator m_elevator = new Elevator();
    private static final Conveyor m_conveyor = new Conveyor();
    //private static final Intake m_intake = new Intake();

    private static final ShuffleboardTabManager m_shuffleboardTabManager = new ShuffleboardTabManager(m_swerve, null);
    
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
        Controlboard.getZeroGyro().onTrue(Commands.runOnce(() -> m_swerve.resetYaw(AllianceFlippable.getForwardRotation())));
        Controlboard.getResetPose().onTrue(Commands.runOnce(() -> m_swerve.resetPose(new Pose2d(FieldConstants.RED_PODIUM, AllianceFlippable.getForwardRotation()))));
        //Controlboard.getBTestButton().whileTrue(new FeedForwardCharacterization(m_elevator, true, new FeedForwardCharacterizationData("Elevator"), m_elevator::setElevatorVoltage, m_elevator::getElevatorVelocity, m_elevator::getElevatorAcceleration));

        /* Conveyor */
        Controlboard.getEjectShooter().whileTrue(new ConveyorCommand(m_conveyor, -0.85, false)).onFalse(Commands.runOnce(() -> m_conveyor.stop()));
        Controlboard.getManualShooter().whileTrue(new ConveyorCommand(m_conveyor, -ConveyorConstants.AMP_TRAP_SPEED, false)).onFalse(Commands.runOnce(() -> m_conveyor.stop()));

        /* Elevator */
        //Controlboard.getManualMode().toggleOnTrue(new ElevatorManualCommand(m_elevator, Controlboard.getManualElevator_Output())).toggleOnFalse(Commands.runOnce(() -> m_elevator.stop()));

        /* Pivot */
        // Controlboard.getManualMode().toggleOnTrue(new PivotManualCommand(m_pivot, Controlboard.getManualPivot_Output()));

        /* Pivot AND Elevator */
        // Controlboard.setPosition_Home().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_HOME_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_HOME_ANGLE));
        // Controlboard.setPosition_Subwoofer().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_SUBWOOFER_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_SUBWOOFER_ANGLE));
        // Controlboard.setPosition_Amp().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_AMP_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_AMP_ANGLE));
        // Controlboard.setPosition_Trap().toggleOnTrue(new ElevatorTargetCommand(m_elevator, ElevatorConstants.ELEVATOR_TRAP_POSITION)).toggleOnTrue(new PivotTargetCommand(m_pivot, PivotConstants.PIVOT_TRAP_ANGLE));

        /* Shooter */
        //Controlboard.getManualShooter().whileTrue(new ShooterManualCommand(m_shooter, ShooterConstants.SHOOTER_SHOOT_OUTPUT)).onFalse(Commands.runOnce(() -> m_shooter.stop()));
        //Controlboard.getEjectShooter().whileTrue(new ShooterManualCommand(m_shooter, ShooterConstants.SHOOTER_EJECT_OUTPUT)).onFalse(Commands.runOnce(() -> m_shooter.stop()));

        /* Automation */
        // Controlboard.getAutoPrepShot().toggleOnTrue(new AutoPrepCommand(m_pivot, m_elevator, m_shooter, m_swerve, Controlboard.getDefense().getAsBoolean(), getAlliance())).toggleOnTrue(new FacePointCommand(m_swerve, Controlboard.getTranslation(), AllianceFlippable.Translation2d(FieldConstants.BLUE_SPEAKER_OPENING, FieldConstants.RED_SPEAKER_OPENING), true));
        // Controlboard.getAutoFire().onTrue(new ConveyorAutoFireCommand(m_swerve, m_conveyor, m_shooter, m_pivot, m_elevator));
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
        //Controlboard.getManualIntake().whileTrue(new IntakeCommand(m_intake, IntakeConstants.INTAKE_SPEED, true)).whileTrue(new ConveyorManualCommand(m_conveyor, ConveyorConstants.TRANSFER_SPEED));
        //Controlboard.getEjectIntake().whileTrue(new IntakeCommand(m_intake, IntakeConstants.EJECT_SPEED, true)).whileTrue(new ConveyorManualCommand(m_conveyor, ConveyorConstants.AMP_TRAP_SPEED));
        //Controlboard.getAutoPickup().whileTrue(new FaceGamePieceCommand(m_swerve, Controlboard.getTranslation(), SwerveConstants.VISION_ROTATION_CONSTANTS));

        Controlboard.getManualMode().toggleOnTrue(new ClimberManualCommand(m_climber, Controlboard.getManualClimber()));
    }

    private void configDefaultCommands() { 
        /* Sets the default command for the swerve subsystem */
        /* m_swerve.setDefaultCommand(
            new DriveCommand(
                m_swerve,
                Controlboard.getTranslation(),
                Controlboard.getRotation(),
                Controlboard.getFieldCentric()
            ) 
        ); */
    }

    /**
     * Configures auto. 
     * Configure default auto and named commands with configure(Command defaultAuto, NamedCommand... namedCommands)<p>
     * Add auto routines with addCommands(Command... commands)
     */
    private void configAuto() {
        Autonomous.configure(
            Commands.none().withName("Do Nothing"),
            new PPEvent("StartIntake", Commands.none()),//new IntakeAutoCommand(m_intake, IntakeConstants.INTAKE_SPEED, false)),
            new PPEvent("StopIntake", Commands.none()),
            new PPEvent("StartAimAtSpeaker", m_swerve.overrideRotationTargetCommand(ScreamUtil.calculateYawToPose(m_swerve.getPose(), AllianceFlippable.getTargetSpeaker()))),
            new PPEvent("StopAimAtSpeaker", m_swerve.overrideRotationTargetCommand(null))
        );

        Autonomous.addRoutines(
            //Routines.Close4(m_swerve).withName("Close4"),
            //Routines.AmpSide6(m_swerve).withName("AmpSide6"),
            //Routines.SourceSide4(m_swerve).withName("SourceSide4")
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
        //m_swerve.stopAll();
        OrchestraUtil.stop();
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