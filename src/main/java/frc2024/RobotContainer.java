package frc2024;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2024.Constants.StabilizerConstants;
import frc2024.Constants.ConveyorConstants;
import frc2024.Constants.ElevatorConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.LEDConstants;
import frc2024.Constants.PivotConstants;
import frc2024.Constants.ShooterConstants;
import frc2024.auto.Autonomous;
import frc2024.auto.Autonomous.PPEvent;
import frc2024.auto.Routines;
import frc2024.commands.ClimbSequence;
import frc2024.commands.Feed;
import frc2024.commands.FeedForwardCharacterization;
import frc2024.commands.GoHome;
import frc2024.commands.ShooterIdle;
import frc2024.commands.SmartShootSequence;
import frc2024.commands.PoseShooting;
import frc2024.commands.SuperstructureToPosition;
import frc2024.commands.intake.AutoIntakeFloor;
import frc2024.commands.intake.IntakeFloor;
import frc2024.commands.swerve.TeleopDrive;
import frc2024.commands.swerve.SnappedDrive;
import frc2024.controlboard.Controlboard;
import frc2024.dashboard.ShuffleboardTabManager;
import frc2024.subsystems.Stabilizers;
import frc2024.subsystems.Conveyor;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Intake;
import frc2024.subsystems.LED;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Swerve;

public class RobotContainer {
    
    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();
    private static final Stabilizers m_stabilizers = new Stabilizers();
    private static final Shooter m_shooter = new Shooter();
    private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    private static final Conveyor m_conveyor = new Conveyor();
    private static final Intake m_intake = new Intake();
    private static final LED m_led = new LED();

    private static final ShuffleboardTabManager m_shuffleboardTabManager = new ShuffleboardTabManager(m_swerve, m_stabilizers, m_conveyor, m_elevator, m_intake, m_pivot, m_shooter);

    public static SuperstructureState currentState = SuperstructureState.HOME;

    public static ClimbSequence m_climbSequence = new ClimbSequence(Controlboard.getTranslation(), Controlboard.getRotation(), Controlboard.getManualElevatorOutput(true), m_swerve, m_elevator, m_pivot, m_stabilizers, m_led);

    public static boolean isRunningPath = false;
    
    /**
     * Configures the basic robot systems, such as Shuffleboard, autonomous, default commands, and button bindings.
     */
    public RobotContainer() {
        m_shuffleboardTabManager.addTabs(false);
        configButtonBindings();
        configDefaultCommands();
    }

    /**
     * Configures button bindings from Controlboard.
     */
    private void configButtonBindings() {
        Controlboard.zeroGyro().onTrue(Commands.runOnce(() -> m_swerve.resetEstimatedHeading(AllianceFlipUtil.getForwardRotation())).ignoringDisable(true));
        Controlboard.resetPose_Apriltag().onTrue(Commands.runOnce(() -> m_swerve.resetPose_Apriltag()).ignoringDisable(true));
        Controlboard.driverController_Command.povLeft().whileTrue(new FeedForwardCharacterization(m_shooter, m_shooter::setShooterVoltage, m_shooter::getMotorVelocity));

        Controlboard.snapAnglePresent()
            .whileTrue(
                new SnappedDrive(
                    m_swerve, 
                    Controlboard.getTranslation(), 
                    Controlboard.getSnapAngle(), 
                    Controlboard.getSlowMode())
            );
        
        /* Conveyor */
        Controlboard.clearNote()
            .whileTrue(
                new SuperstructureToPosition(SuperstructureState.EJECT, m_elevator, m_pivot)
                    .alongWith(m_stabilizers.positionCommand(StabilizerConstants.DOWN_POSITION)))
            .onFalse(
                new WaitCommand(0.4).andThen(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake))
                    .alongWith(m_stabilizers.positionCommand(StabilizerConstants.UP_POSITION)));

        /* Elevator */
        Controlboard.manualMode().whileTrue(m_elevator.voltageCommand(Controlboard.getManualElevatorOutput(false)));
        Controlboard.resetElevatorHeight().onTrue(Commands.runOnce(() -> m_elevator.zeroPosition()).ignoringDisable(true));

        /* Pivot */
        Controlboard.manualMode()
            .whileTrue(m_pivot.dutyCycleCommand(Controlboard.getManualPivotOutput()));
        Controlboard.resetPivotAngle().onTrue(Commands.runOnce(() -> m_pivot.resetToAbsolute()).ignoringDisable(true));
        

        /* Pivot AND Elevator */
        Controlboard.goToHomePosition()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.HOME)
                    .andThen(
                        new GoHome(true, m_pivot, m_elevator, m_conveyor, m_intake)));
                    
        Controlboard.goToHomePositionEndgame()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.HOME_ENDGAME)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.HOME_ENDGAME, m_elevator, m_pivot)
                            .until(() -> superstructureAtTarget())));

        Controlboard.goToSubwooferPosition()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.SUBWOOFER)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.SUBWOOFER, m_elevator, m_pivot)
                            .alongWith(m_shooter.velocityCommand(ShooterConstants.SUBWOOFER_VELOCITY)).andThen(m_elevator.heightCommand(ElevatorConstants.HOME_HEIGHT))))
            .onFalse(new GoHome(true, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.goToAmpPosition()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.AMP)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.AMP, m_elevator, m_pivot)))
            .onFalse(
                new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake).alongWith(new PrintCommand("GoHome")));

        Controlboard.goToPodiumPosition()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.PODIUM)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.PODIUM, m_elevator, m_pivot)
                            .alongWith(m_shooter.velocityCommand(ShooterConstants.PODIUM_VELOCITY)))
                                .alongWith(new SnappedDrive(m_swerve, Controlboard.getTranslation(), () -> AllianceFlipUtil.Number(-18.0, -162.0), Controlboard.getSlowMode())))
            .onFalse(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.goToChainPosition()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.CHAIN)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.CHAIN, m_elevator, m_pivot)
                            .alongWith(m_shooter.velocityCommand(ShooterConstants.CHAIN_VELOCITY))))
            .onFalse(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.goToSubwooferPositionDefended()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.SUBWOOFER_DEFENDED)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.SUBWOOFER_DEFENDED, m_elevator, m_pivot)
                            .alongWith(m_shooter.velocityCommand(ShooterConstants.SUBWOOFER_DEFENDED_VELOCITY))))
            .onFalse(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.goToPodiumPositionDefended()
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.PODIUM_DEFENDED)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.PODIUM_DEFENDED, m_elevator, m_pivot)
                            .alongWith(m_shooter.velocityCommand(ShooterConstants.PODIUM_VELOCITY))))
            .onFalse(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.goToTrapPosition()
            .toggleOnTrue(
                new InstantCommand(() -> currentState = SuperstructureState.TRAP_CHAIN)
                    .andThen(
                        new SuperstructureToPosition(SuperstructureState.TRAP_CHAIN, m_elevator, m_pivot)
                            .alongWith(m_shooter.stopCommand())));

        Controlboard.autoClimb()
            .toggleOnTrue(
                new InstantCommand(() -> currentState = SuperstructureState.TRAP_CHAIN)
                    .andThen(
                        m_elevator.heightCommand(ElevatorConstants.TRAP_CHAIN_HEIGHT))
                    .alongWith(
                        m_pivot.angleCommand(PivotConstants.HOME_ANGLE))
                    .alongWith(
                        m_stabilizers.outputCommand(StabilizerConstants.OUT_OUTPUT)
                            .withTimeout(1)
                            .andThen(m_stabilizers.stopCommand())));

        Controlboard.advanceClimbSequence().and(() -> DriverStation.getMatchTime() <= 30.0)
            .onTrue(
                Commands.runOnce(() -> 
                    {
                        currentState = SuperstructureState.TRAP_CHAIN;
                        m_climbSequence.schedule();
                    })
                    .andThen(Commands.runOnce(() -> m_climbSequence.advance())));

        /* Shooter */        
        Controlboard.shooterIntoConveyor()
            .whileTrue(
                m_shooter.dutyCycleCommand(-ShooterConstants.EJECT_OUTPUT))
            .onFalse(m_shooter.stopCommand());

        Controlboard.stopFlywheel()
            .onTrue(Commands.run(() -> m_shooter.stop(), m_shooter));

        /* Automation */
        Controlboard.autoFire().and(Controlboard.virtualAutoFire())
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.AUTO_FIRE)
                    .andThen(
                        new SmartShootSequence(Controlboard.getTranslation(), false, false, true, m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_led)))
            .onFalse(new GoHome(true, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.autoFire().and(Controlboard.virtualAutoFire().negate())
            .whileTrue(
                new InstantCommand(() -> currentState = SuperstructureState.AUTO_FIRE)
                    .andThen(
                        new ConditionalCommand(
                            new Feed(Controlboard.getTranslation(), m_swerve, m_pivot, m_elevator, m_shooter, m_conveyor, m_intake, m_led).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
                            new PoseShooting(Controlboard.getTranslation(), Controlboard.defendedMode(), m_swerve, m_pivot, m_elevator, m_shooter, m_conveyor, m_intake, m_led).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), 
                            () -> ScreamUtil.calculateDistanceToTranslation(() -> m_swerve.getEstimatedPose().getTranslation(), () -> AllianceFlipUtil.getTargetSpeaker().getTranslation()).getAsDouble() >= Units.feetToMeters(25.0))))
            .onFalse(new GoHome(true, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.intakeFromFloor().and(new Trigger(m_conveyor.hasPiece(false)).negate())
                .whileTrue(
                    new IntakeFloor(m_elevator, m_pivot, m_conveyor, m_intake, m_led, () -> false)
                )
                    .onFalse(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake));

        new Trigger(m_conveyor.hasPiece(false))
            .and(Controlboard.intakeFromFloor().or(Controlboard.intakeFromFloorEndgame()).or(Controlboard.driverController_Command.rightStick()))
            .onTrue(
                Controlboard.driverRumbleCommand(RumbleType.kBothRumble, 0.8, Units.millisecondsToSeconds(300))
                    .alongWith(m_led.strobeCommand(Color.kGreen, 0.1).withTimeout(1)));

        (Controlboard.intakeFromFloorEndgame().or(Controlboard.driverController_Command.rightStick())).and(new Trigger(m_conveyor.hasPiece(true)).negate())
                .whileTrue(
                    new InstantCommand(() -> currentState = SuperstructureState.HOME_ENDGAME)
                    .andThen(
                        new IntakeFloor(m_elevator, m_pivot, m_conveyor, m_intake, m_led, () -> true))
                )
                .onFalse(
                    new GoHome(true, m_pivot, m_elevator, m_conveyor, m_intake)
                );

        Controlboard.intakeOverride().whileTrue(m_conveyor.dutyCycleCommand(ConveyorConstants.TRANSFER_OUTPUT)
            .alongWith(m_intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT)))
                .onFalse(m_conveyor.stopCommand().alongWith(m_intake.stopCommand()));

        Controlboard.intakeOverrideEndgame().whileTrue(m_conveyor.dutyCycleCommand(ConveyorConstants.AMP_OUTPUT)
            .alongWith(m_intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT)))
                .onFalse(m_conveyor.stopCommand().alongWith(m_intake.stopCommand()));

        Controlboard.eject()
            .whileTrue(
                new SuperstructureToPosition(SuperstructureState.EJECT, m_elevator, m_pivot)
                .alongWith(
                    m_intake.dutyCycleCommand(IntakeConstants.EJECT_OUTPUT)
                        .alongWith(m_conveyor.dutyCycleCommand(ConveyorConstants.AMP_OUTPUT))
                        .alongWith(m_shooter.dutyCycleCommand(ShooterConstants.EJECT_OUTPUT))))
            .onFalse(new GoHome(false, m_pivot, m_elevator, m_conveyor, m_intake));

        Controlboard.trapAdjustUp()
            .whileTrue(
                m_conveyor.dutyCycleCommand(-0.075 - 0.02))
            .onFalse(m_conveyor.stopCommand());

        Controlboard.trapAdjustDown()
            .whileTrue(
                m_conveyor.dutyCycleCommand(0.075 + 0.02))
            .onFalse(m_conveyor.stopCommand());

        Controlboard.score()
            .whileTrue(m_conveyor.scoreCommand())
                .onFalse(m_conveyor.stopCommand());

        Controlboard.elevatorDown_MAX().whileTrue(
            m_elevator.voltageCommand(-12.0)
        ).onFalse(
            m_elevator.stopCommand()
        );

        Controlboard.stopClimbSequence().debounce(0.5).onTrue(
            Commands.runOnce(() -> m_climbSequence.cancel())
        );

        Controlboard.defendedMode()
            .onTrue(new InstantCommand(() -> m_led.setDefaultCommand(m_led.strobeCommand(Color.kRed, 0.07)))
                .andThen(m_led.strobeCommand(Color.kRed, 0.07)))
            .onFalse(new InstantCommand(() -> m_led.setDefaultCommand(m_led.waveCommand(() -> (Color) AllianceFlipUtil.Object(Color.kBlue, Color.kRed), () -> Color.kBlack, 22, 2)))
                .andThen(m_led.waveCommand(() -> (Color) AllianceFlipUtil.Object(Color.kBlue, Color.kRed), () -> Color.kBlack, LEDConstants.STRIP_LENGTH / 3.0, 1.5)));
        
        /* Stabilizers */
        new Trigger(Controlboard.endGameMode())
            .whileTrue(
                m_stabilizers.outputCommand(Controlboard.getManualClimberOutput()))
            .onTrue(new InstantCommand(() -> m_led.setDefaultCommand(m_led.rainbowCommand(10, 1.5)))
                .andThen(m_led.rainbowCommand(10, 1.5)))
            .onFalse(new InstantCommand(() -> m_led.setDefaultCommand(m_led.waveCommand(() -> (Color) AllianceFlipUtil.Object(Color.kBlue, Color.kRed), () -> Color.kBlack, 22, 2)))
                .andThen(m_led.waveCommand(() -> (Color) AllianceFlipUtil.Object(Color.kBlue, Color.kRed), () -> Color.kBlack, LEDConstants.STRIP_LENGTH / 3.0, 1.5)));
    }

    private void configDefaultCommands() { 
        /* Sets the default command for the swerve subsystem */
        m_swerve.setDefaultCommand(
            new TeleopDrive(
                m_swerve,
                Controlboard.getTranslation(),
                Controlboard.getRotation(),
                Controlboard.getFieldCentric(),
                Controlboard.getSlowMode())
        );

        m_shooter.setDefaultCommand(
            new ShooterIdle(Controlboard.endGameMode(), m_swerve, m_conveyor, m_shooter));

        m_led.setDefaultCommand(
            m_led.waveCommand(() -> (Color) AllianceFlipUtil.Object(Color.kBlue, Color.kRed), () -> Color.kBlack, LEDConstants.STRIP_LENGTH / 3.0, 1.5)
        );
    }

    /**
     * Configures auto. 
     * Configure default auto and named commands with configure(Command defaultAuto, NamedCommand... namedCommands)<p>
     * Add auto routines with addCommands(Command... commands)
     */
    public static void configAuto() {
        Autonomous.configure(
            Commands.none().withName("Do Nothing"),
            new PPEvent("RunIntake", new AutoIntakeFloor(m_elevator, m_pivot, m_conveyor, m_intake, m_led)),
            new PPEvent("RunIntakeContinuous", m_conveyor.dutyCycleCommand(ConveyorConstants.SHOOT_OUTPUT).alongWith(m_intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT))),
            new PPEvent("RunShooterHigh", m_shooter.velocityCommand(4500)),
            new PPEvent("Shoot", m_conveyor.dutyCycleCommand(ConveyorConstants.SHOOT_OUTPUT).withTimeout(1).andThen(m_conveyor.stopCommand())),
            new PPEvent("StartIntake_Conveyor", m_conveyor.dutyCycleAutoEndCommand(ConveyorConstants.SHOOT_OUTPUT)
                .alongWith(
                    m_intake.dutyCycleCommand(IntakeConstants.INTAKE_OUTPUT)
                )),
            new PPEvent("StopIntake_Conveyor", m_conveyor.stopCommand().alongWith(m_intake.stopCommand())),
            new PPEvent("Relocalize", new InstantCommand(() -> m_swerve.resetOdometryToEstimated()))
        );

        Autonomous.addRoutines(
            Routines.Amp4Close(m_swerve, m_shooter, m_elevator, m_pivot, m_conveyor, m_intake, m_led).withName("Amp_4_Close"),
            Routines.Amp5_1Center(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Amp_5.5_Close&Center"),
            Routines.SweepSource(m_swerve, m_pivot, m_shooter, m_conveyor, m_intake).withName("SweepSource"),
            Routines.Sweep3_Source(m_swerve, m_pivot, m_elevator, m_shooter, m_conveyor, m_intake, m_led).withName("Sweep3_Source"),
            Routines.Amp5Center_2(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("SubSide_4_1Close&Center"),
            Routines.Amp5_Stage(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Amp_5_Close&Center"),
            Routines.Amp5_NoStage_2(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Amp5_NoStage_2"),
            Routines.Amp5_1Center_Piece1(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Amp5.5_Piece1"),
            Routines.Amp6_SplitOff(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Amp6_SplitOff"),
            Routines.Source3_NoStage(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Source3_Center&NoStage"),
            Routines.Source3_Stage(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Source3_Stage"),
            Routines.Source2_1Sweep(m_swerve, m_pivot, m_elevator, m_shooter, m_conveyor, m_intake, m_led).withName("Source2_Sweep1"),
            Routines.Amp4Bypass_WithSkip_Relocalize(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Amp4_Bypass"),
            Routines.Center2(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Center2"),
            Routines.Front5Sub_CenterRush(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Front5Sub_CenterRush"),
            Routines.Source3_Center2To1_WithSkip(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Source3_Center2To1"),
            Routines.Source3_Center1To2_WithSkip(m_swerve, m_elevator, m_pivot, m_shooter, m_conveyor, m_intake, m_led).withName("Source3_Center1To2"),
            Routines.Leave(m_swerve, 2.0).withName("Leave"),
            Routines.testAuto(m_swerve).withName("test")
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

    public static Stabilizers getStabilizers(){
        return m_stabilizers;
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

    public static LED getLED(){
        return m_led;
    }

    public static Command goHome(String test){
        return new InstantCommand(() -> currentState = SuperstructureState.HOME)
                .alongWith(new PrintCommand(test))
                .andThen(
                    new SuperstructureToPosition(SuperstructureState.HOME, m_elevator, m_pivot)
                        .until((() -> superstructureAtTarget()))
                        .alongWith(
                            new ParallelCommandGroup(
                                m_conveyor.stopCommand(), 
                                m_intake.stopCommand())));
    }

    public static boolean superstructureAtTarget(){
        return m_elevator.getElevatorAtTarget().getAsBoolean() && m_pivot.getPivotAtTarget().getAsBoolean();
    }

    public static boolean forwardPivotLimit(){
        boolean elevatorAtMaxHeight = m_elevator.getElevatorHeight() >= ElevatorConstants.MAX_HEIGHT - ElevatorConstants.TARGET_THRESHOLD;
        boolean pivotAtMaxPivot = m_pivot.getPivotAngle().getDegrees() >= PivotConstants.FORWARD_SOFT_LIMIT_ENDGAME.getDegrees();
        return elevatorAtMaxHeight && pivotAtMaxPivot;
    }
    
    public static boolean reversePivotLimit(){
        boolean elevatorAtMaxHeight = m_elevator.getElevatorHeight() >= ElevatorConstants.MAX_HEIGHT - ElevatorConstants.TARGET_THRESHOLD;
        boolean pivotAtMaxPivot = m_pivot.getPivotAngle().getDegrees() <= PivotConstants.REVERSE_SOFT_LIMIT_ENDGAME.getDegrees();
        return elevatorAtMaxHeight && pivotAtMaxPivot;
    }

    public static Supplier<SuperstructureState> getCurrentState(){
        return () -> currentState;
    }

    public static void logOutputs(){
        Logger.recordOutput("RobotState/SuperstructureState", getCurrentState().get());
        Logger.recordOutput("RobotState/TrapForwardLimitExceeded", forwardPivotLimit());
        Logger.recordOutput("RobotState/TrapReverseLimitExceeded", reversePivotLimit());
    }

    public static void stopAll(){
        m_stabilizers.stop();
        m_shooter.stop();
        m_pivot.stop();
        m_elevator.stop();
        m_conveyor.stop();
        m_intake.stop();
        m_swerve.stopAll();
        OrchestraUtil.stop();
    }
    
    public static void setAllNeutralModes(NeutralModeValue mode){
        m_stabilizers.setNeutralMode(mode);
        m_shooter.setNeutralMode(mode);
        m_pivot.setNeutralMode(mode);
        m_elevator.setNeutralMode(mode);
        m_conveyor.setNeutralMode(mode);
        m_intake.setNeutralMode(mode);
        m_swerve.setNeutralModes(mode, mode);
    }
}