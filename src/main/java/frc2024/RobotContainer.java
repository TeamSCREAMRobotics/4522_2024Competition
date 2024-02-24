package frc2024;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.Optional;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.ElevatorPivotPosition;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.Constants.SwerveConstants;
import frc2024.auto.Autonomous;
import frc2024.auto.Autonomous.PPEvent;
import frc2024.auto.Routines;
import frc2024.commands.AutoFire;
import frc2024.commands.FeedForwardCharacterization;
import frc2024.commands.SuperstructureToPosition;
import frc2024.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc2024.commands.intake.AutoIntakeFloor;
import frc2024.commands.intake.IntakeFloor;
import frc2024.commands.swerve.TeleopDrive;
import frc2024.commands.swerve.DriveToPose;
import frc2024.commands.swerve.FacePoint;
import frc2024.commands.swerve.FaceVisionTarget;
import frc2024.controlboard.Controlboard;
import frc2024.dashboard.ShuffleboardTabManager;
import frc2024.subsystems.Climber;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.swerve.Swerve;

public class RobotContainer {
    
    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();
    private static final Climber m_climber = new Climber();
    private static final Shooter m_shooter = new Shooter();
    private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    private static final Conveyor m_conveyor = new Conveyor();
    private static final Intake m_intake = new Intake();

    private static final ShuffleboardTabManager m_shuffleboardTabManager = new ShuffleboardTabManager(m_swerve, m_climber, m_conveyor, m_elevator, m_intake, m_pivot, m_shooter);

    private static Alliance m_alliance;
    
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
        Controlboard.zeroGyro().onTrue(Commands.runOnce(() -> m_swerve.resetYaw(AllianceFlippable.getForwardRotation())));
        Controlboard.resetPose().onTrue(Commands.runOnce(() -> m_swerve.resetPose(new Pose2d(FieldConstants.RED_PODIUM, new Rotation2d()))));/* m_elevator.zeroPosition()).ignoringDisable(true) );*/

        /* Conveyor */
        Controlboard.ejectThroughShooter().whileTrue(m_conveyor.outputCommand(-0.85)).onFalse(m_conveyor.stopCommand());
        //POV Down
        Controlboard.manuallyShoot().whileTrue(m_conveyor.outputCommand(0.45)).onFalse(m_conveyor.stopCommand());
        //POV Up

        /* Elevator */
        Controlboard.manualMode().toggleOnTrue(m_elevator.voltageCommand(Controlboard.getManualElevatorOutput())).toggleOnFalse(Commands.runOnce(() -> m_elevator.stop()));
        Controlboard.resetElevatorHeight().onTrue(Commands.runOnce(() -> m_elevator.zeroPosition()).ignoringDisable(true));

        /* Pivot */
        Controlboard.manualMode().toggleOnTrue(m_pivot.dutyCycleCommand(Controlboard.getManualPivotOutput()));
        Controlboard.resetPivotAngle().onTrue(Commands.runOnce(() -> m_pivot.zeroPivot()).ignoringDisable(true));
        //Right Y

        /* Pivot AND Elevator */
        Controlboard.goToHomePosition().whileTrue(
            new SuperstructureToPosition(ElevatorPivotPosition.HOME, m_elevator, m_pivot)
                .until(() -> superstructureAtTarget()));

        Controlboard.goToSubwooferPosition()
            .whileTrue(new SuperstructureToPosition(ElevatorPivotPosition.SUBWOOFER, m_elevator, m_pivot).alongWith(m_shooter.velocityCommand(ShooterConstants.SUBWOOFER_VELOCITY)))
            .onFalse(
                new SuperstructureToPosition(ElevatorPivotPosition.HOME, m_elevator, m_pivot).alongWith(m_shooter.velocityCommand(ShooterConstants.RESTING_VELOCITY))
                    .until((() -> superstructureAtTarget())));

        Controlboard.goToAmpPosition()
            .whileTrue(new SuperstructureToPosition(ElevatorPivotPosition.AMP, m_elevator, m_pivot))
            .onFalse(
                new SuperstructureToPosition(ElevatorPivotPosition.HOME, m_elevator, m_pivot)
                    .until((() ->  superstructureAtTarget())));

        Controlboard.goToPodiumPosition()
            .whileTrue(new SuperstructureToPosition(ElevatorPivotPosition.PODIUM_HIGH, ElevatorPivotPosition.PODIUM_LOW, m_elevator, m_pivot, Controlboard.defendedMode())
                    .alongWith(m_shooter.velocityCommand(ShooterConstants.PODIUM_VELOCITY)))
                        .onFalse(
                            new SuperstructureToPosition(ElevatorPivotPosition.HOME, m_elevator, m_pivot)
                                .until((() ->  superstructureAtTarget())).alongWith(m_shooter.velocityCommand(ShooterConstants.RESTING_VELOCITY)));

        Controlboard.goToTrapPosition()
            .whileTrue(new SuperstructureToPosition(ElevatorPivotPosition.TRAP_CHAIN, m_elevator, m_pivot))
            .onFalse(
                new SuperstructureToPosition(ElevatorPivotPosition.HOME, m_elevator, m_pivot)
                    .until((() -> superstructureAtTarget())));

        /* Shooter */
        Controlboard.testA()
            .whileTrue(
                m_shooter.velocityCommand(0))
                    .onFalse(m_shooter.stopCommand());
        //A button
        
        Controlboard.ejectThroughShooter()
            .whileTrue(
                m_shooter.dutyCycleCommand(ShooterConstants.EJECT_OUTPUT))
            .onFalse(m_shooter.stopCommand());

        Controlboard.stopFlywheel().toggleOnTrue(m_shooter.stopCommand());

        /* Automation */
        Controlboard.autoFire()
            .toggleOnTrue(
                new AutoFire(m_swerve, m_shooter, m_elevator, m_pivot, m_conveyor, Controlboard.defendedMode())
                    .alongWith(new FaceVisionTarget(m_swerve, Controlboard.getTranslation(), SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER)))
            .onFalse(
                new SuperstructureToPosition(ElevatorPivotPosition.HOME, m_elevator, m_pivot)
                    .until((() -> superstructureAtTarget())).alongWith(m_shooter.velocityCommand(ShooterConstants.RESTING_VELOCITY).alongWith(m_conveyor.stopCommand().alongWith(m_intake.stopCommand()))));

        /* Controlboard.autoFire()
            .whileTrue(
                new AutoFire(m_swerve, m_shooter, m_elevator, m_pivot, m_conveyor, Controlboard.defendedMode())
                    .deadlineWith(new FacePoint(m_swerve, Controlboard.getTranslation(), AllianceFlippable.getTargetSpeaker().getTranslation(), false, true))); */

        /* Intake */
        Controlboard.intakeFromFloor().and(new Trigger(m_conveyor.hasPiece()).negate())
            .whileTrue(new IntakeFloor(m_elevator, m_pivot, m_conveyor, m_intake));
        
        Controlboard.score()
            .whileTrue(m_conveyor.outputCommand(ConveyorConstants.AMP_TRAP_OUTPUT))
                .onFalse(m_conveyor.stopCommand());

        /* Controlboard.autoPickupFromFloor()
            .whileTrue(
                new AutoIntakeFloor(Controlboard.getTranslation(), m_swerve, m_elevator, m_pivot, m_intake, m_conveyor)
                    .until(() -> m_conveyor.hasPiece())); */

        /* Climber */
        Controlboard.manualMode()
            .toggleOnTrue(
                m_climber.outputCommand(Controlboard.getManualClimberOutput()));
    }

    private void configDefaultCommands() { 
        /* Sets the default command for the swerve subsystem */
        m_swerve.setDefaultCommand(
            new TeleopDrive(
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
     * Add auto routines with addCommands(Command... commands)
     */
    private void configAuto() {
        Autonomous.configure(
            Commands.none().withName("Do Nothing"),
            new PPEvent("StartIntake", new IntakeFloor(m_elevator, m_pivot, m_conveyor, m_intake).withTimeout(3)),//new IntakeAutoCommand(m_intake, IntakeConstants.INTAKE_SPEED, false)),
            new PPEvent("StopIntake", Commands.none()),
            new PPEvent("StartAutoFire", 
                new AutoFire(m_swerve, m_shooter, m_elevator, m_pivot, m_conveyor, () -> false)
                    .alongWith(new FaceVisionTarget(m_swerve, new DoubleSupplier[2], SwerveConstants.SNAP_CONSTANTS, Limelight.SHOOTER))),
            new PPEvent("StopAutoFire", 
                m_shooter.stopCommand()
                    .alongWith(m_conveyor.stopCommand())
                    .alongWith(m_intake.stopCommand())
                    .alongWith(new InstantCommand(() -> m_swerve.getCurrentCommand().cancel())))
        );

        Autonomous.addRoutines(
            Routines.Close4(m_swerve, m_shooter, m_elevator, m_pivot, m_conveyor).withName("Close4"),
            Routines.AmpSide6(m_swerve, m_elevator, m_pivot, m_intake, m_conveyor).withName("AmpSide6"),
            Routines.SourceSide4(m_swerve).withName("SourceSide4")
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

    public static Climber getClimber(){
        return m_climber;
    }

    public static Shooter getShooter(){
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
    }

    public static Intake getIntake(){
        return m_intake;
    }

    /**
     * Retrieves the current Alliance as detected by the DriverStation. 
     * Use this opposed to DriverStation.getAlliance().
     * @return The current Alliance.
     */
    public static Alliance getAlliance(){
        return m_alliance;
    }

    public static void setAlliance(Alliance alliance){
        m_alliance = alliance;
    }

    public static boolean superstructureAtTarget(){
        return m_elevator.getElevatorAtTarget() && m_pivot.getPivotAtTarget();
    }

    public static void stopAll(){
        m_climber.stop();
        m_shooter.stop();
        m_pivot.stop();
        m_elevator.stop();
        m_conveyor.stop();
        m_intake.stop();
        m_swerve.stopAll();
        OrchestraUtil.stop();
    }
    
    public static void setAllNeutralModes(NeutralModeValue mode){
        //m_shooter.setNeutralMode(mode);
        //m_pivot.setNeutralMode(mode);
        //m_elevator.setNeutralMode(mode);
        //m_conveyor.setNeutralMode(mode);
        //m_intake.setNeutralMode(mode);
        //m_swerve.setNeutralModes(mode, mode);
    }
}