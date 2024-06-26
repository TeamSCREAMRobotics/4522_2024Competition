package frc2024.controlboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class that contains button bindings.
 * 
 * Controlboard allows easy reference of custom button associations.
 */
public class Controlboard{

    public static final double STICK_DEADBAND = 0.05;
    public static final double TRIGGER_DEADBAND = 0.10;
    public static final Rotation2d SNAP_TO_POLE_THRESHOLD = Rotation2d.fromDegrees(7.0);

    public static final CommandXboxController driverController_Command = new CommandXboxController(0);
    public static final CommandXboxController operatorController_Command = new CommandXboxController(1);
    public static final Buttonboard buttonBoard = new Buttonboard(2, 3);

    private static boolean driverDefended = false;
    public static boolean fieldCentric = true;
    static{
        driverController_Command.x().onTrue(Commands.runOnce(() -> driverDefended = !driverDefended));
        driverController_Command.start().onTrue(Commands.runOnce(() -> fieldCentric = !fieldCentric));
    }

    public static Command driverRumbleCommand(RumbleType type, double value, double time){
        return Commands.startEnd(
            () -> driverController_Command.getHID().setRumble(type, value),
            () -> driverController_Command.getHID().setRumble(type, 0.0))
            .withTimeout(time);
    }

    public static Command operatorRumbleCommand(RumbleType type, double value, double time){
        return Commands.startEnd(
            () -> operatorController_Command.getHID().setRumble(type, value),
            () -> operatorController_Command.getHID().setRumble(type, 0.0))
            .withTimeout(time);
    }

    /**
     * Retrieves the swerve translation from the driver controller.
     * 
     * @return A DoubleSupplier array representing the x and y values from the controller.
     */
    public static DoubleSupplier[] getTranslation(){
        return new DoubleSupplier[]{
            () -> -MathUtil.applyDeadband(driverController_Command.getLeftY(), STICK_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController_Command.getLeftX(), STICK_DEADBAND),
        };
    }

    private static DoubleSupplier[] snapTranslationToPole(DoubleSupplier[] translation){
        Translation2d temp = new Translation2d(translation[0].getAsDouble(), translation[1].getAsDouble());
		for(int i = 0; i < 360; i+=90){
			if(Math.abs(temp.getAngle().minus(Rotation2d.fromDegrees(i)).getDegrees()) < SNAP_TO_POLE_THRESHOLD.getDegrees()){
                Translation2d snapped = new Translation2d(temp.getNorm(), Rotation2d.fromDegrees(i));
                return new DoubleSupplier[]{
                    () -> snapped.getX(),
                    () -> snapped.getY()
                };
            }
		}
        return translation;
	}

    public static DoubleSupplier getSnapAngle(){
        return () -> {
            if(driverController_Command.getHID().getAButton()){
                return 90.0;
            } else if(driverController_Command.getHID().getYButton()){
                return AllianceFlipUtil.Number(0.0, 180.0);
            } else {
                return -1.0;
            }
        };
    }

    public static Trigger snapAnglePresent(){
        return new Trigger(() -> getSnapAngle().getAsDouble() != -1.0);
    }

    public static BooleanSupplier getSlowMode(){
        return () -> driverController_Command.getHID().getLeftTriggerAxis() >= TRIGGER_DEADBAND;
    }

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return A DoubleSupplier representing the rotation.
     */
    public static DoubleSupplier getRotation() {
        return () -> -MathUtil.applyDeadband(driverController_Command.getRightX(), STICK_DEADBAND);
    }

    public static Trigger dodgeRight(){
        return driverController_Command.rightStick();
    }

    public static Trigger dodgeLeft(){
        return driverController_Command.leftStick();
    }

    /**
     * Retrieves whether to zero the gyro from the driver controller.
     *
     * @return A Trigger representing the state of the start button.
     */
    public static Trigger zeroGyro() {
        return driverController_Command.back();
    }

    public static Trigger resetPose() {
        return driverController_Command.povUp();
    }

    public static Trigger resetPose_Apriltag() {
        return operatorController_Command.povUp();
    }

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static BooleanSupplier getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        return () -> fieldCentric;
    }

    public static Trigger manualMode(){
    /* Changes how the angle/height of the pivot and elevator are input */
        return new Trigger(() -> buttonBoard.getRawSwitch(1))
            .or(operatorController_Command.povRight());
    }

    /* Automation */
    public static final Trigger prepShot(){
        /* Uses a toggle switch to enable or disable automatic prep shooting */
        return new Trigger(() -> buttonBoard.getRawButton(3));
    }

    public static final Trigger stopClimbSequence(){
        return driverController_Command.b();
    }

    public static final Trigger advanceClimbSequence(){
        return driverController_Command.b();
    }

    public static final Trigger autoFire(){
        /* Uses a toggle switch to enable or disable automatic firing when requirements are met */
        /* return new Trigger(() -> buttonBoard.getRawSwitch(4)); */
        return driverController_Command.leftBumper();
        //return new Trigger(() -> buttonBoard.getRawButton(3));
    }

    public static final Trigger defendedMode(){
        /* Uses a toggle switch to enable or disable wether we are being defended. This allows us to raise our elevator with auto shots */
        return new Trigger(() -> buttonBoard.getRawSwitch(4) || driverDefended);
    }

    public static final Trigger virtualAutoFire(){
        return new Trigger(() -> buttonBoard.getRawSwitch(3));
    }

    public static final BooleanSupplier endGameMode(){
        return () -> buttonBoard.getRawSwitch(2);
    }

    public static final Trigger snapToSpeaker(){
        return driverController_Command.a();
    }

    /* Climber */
    public static final DoubleSupplier getManualClimberOutput(){
        return () -> buttonBoard.getBigSwitchY()*0.10;
    }

    /* Shooter/Conveyor */
    public static final Trigger clearNote(){
        return new Trigger(() -> buttonBoard.getRawButton(1));
    }

    public static final Trigger shooterIntoConveyor(){
        return operatorController_Command.leftBumper();
    }

    public static final Trigger eject(){
        return driverController_Command.povRight();
    }

    public static final Trigger stopFlywheel(){
        return new Trigger(() -> buttonBoard.getRawButton(5));
    }

    /* Pivot */
    public static final DoubleSupplier getManualPivotOutput(){
        return () -> -MathUtil.applyDeadband(operatorController_Command.getRightY(), STICK_DEADBAND)/2;
    }

    public static final Trigger resetPivotAngle(){
        return new Trigger(() -> buttonBoard.getRawButton(4));
    }

    public static final BooleanSupplier increasePivot(){
        return () -> operatorController_Command.getHID().getPOV() == 0.0;
    }

    public static final BooleanSupplier decreasePivot(){
        return () -> operatorController_Command.getHID().getPOV() == 180.0;
    }

    /* Elevator */
    public static final DoubleSupplier getManualElevatorOutput(boolean driverController){
        return () -> driverController
               ? (Math.signum(-driverController_Command.getRightY()) * Math.pow(-MathUtil.applyDeadband(driverController_Command.getRightY(), STICK_DEADBAND), 2.0)) * 8.0
               : (-MathUtil.applyDeadband(operatorController_Command.getLeftY(), STICK_DEADBAND))*6.0;
    }

    public static final Trigger elevatorDown_MAX(){
        return operatorController_Command.a();
    }
    
    public static final Trigger resetElevatorHeight(){
        return new Trigger(() -> buttonBoard.getRawButton(6));
    }

    /* Elevator AND Pivot */    
    public static final Trigger goToChainPosition(){
        return new Trigger(() -> buttonBoard.getRawButton(8));
    }

    public static final Trigger goToHomePosition(){
        return new Trigger(() -> buttonBoard.getRawButton(9)).and(new Trigger(endGameMode()).negate());
    }
    
    public static final Trigger goToHomePositionEndgame(){
        return new Trigger(() -> buttonBoard.getRawButton(9)).and(endGameMode());
    }
    
    public static final Trigger goToSubwooferPosition(){
        return new Trigger(() -> buttonBoard.getRawButton(10)).or(driverController_Command.y()).and(defendedMode().negate());
    }
    
    public static final Trigger goToAmpPosition(){
        return new Trigger(() -> buttonBoard.getRawButton(11)).or(driverController_Command.a());
    }

    public static final Trigger goToPodiumPosition(){
        return new Trigger(() -> buttonBoard.getRawButton(12)).and(defendedMode().negate())
            .and(new Trigger(endGameMode()).negate());
    }

    public static final Trigger goToTrapPosition(){
        return new Trigger(() -> buttonBoard.getRawButton(12)).and(endGameMode());
    }

    public static final Trigger autoClimb(){
        return new Trigger(() -> buttonBoard.getRawButton(2)).and(endGameMode());
    }
    
    public static final Trigger goToSubwooferPositionDefended(){
        return new Trigger(() -> buttonBoard.getRawButton(10)).or(driverController_Command.y()).and(defendedMode());
    }

    public static final Trigger goToPodiumPositionDefended(){
        return new Trigger(() -> buttonBoard.getRawButton(12)).and(defendedMode())
            .and(new Trigger(endGameMode()).negate());
    }

    /* Intake */
    public static final Trigger intakeFromFloor(){
        return driverController_Command.rightTrigger(TRIGGER_DEADBAND).and(new Trigger(endGameMode()).negate());
    }

    public static final Trigger intakeFromFloorEndgame(){
        return driverController_Command.rightTrigger(TRIGGER_DEADBAND).and(endGameMode());
    }

    public static final Trigger score(){
        return driverController_Command.rightBumper();
    }

    public static final Trigger autoPickupFromFloor(){
        return driverController_Command.leftBumper();
    }

    public static final Trigger trapAdjustDown(){
        return driverController_Command.povDown();
    }

    public static final Trigger trapAdjustUp(){
        return driverController_Command.povUp();
    }

    public static final Trigger intakeOverride(){
        return new Trigger(() -> buttonBoard.getRawButton(7)).and(new Trigger(endGameMode()).negate());
    }

    public static final Trigger intakeOverrideEndgame(){
        return new Trigger(() -> buttonBoard.getRawButton(7)).and(endGameMode());
    }
}
