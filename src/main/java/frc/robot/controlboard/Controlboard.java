package frc.robot.controlboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private static final CommandXboxController driverController = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    private static boolean fieldCentric = true;

    /**
     * Retrieves the swerve translation from the driver controller.
     * 
     * @return A DoubleSupplier array representing the x and y values from the controller.
     */
    public static DoubleSupplier[] getTranslation(){
        return new DoubleSupplier[] {
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), STICK_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), STICK_DEADBAND)
        };
    }

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return A DoubleSupplier representing the rotation.
     */
    public static DoubleSupplier getRotation() {
        return () -> -MathUtil.applyDeadband(driverController.getRightX(), STICK_DEADBAND);
    }

    /**
     * Retrieves whether to zero the gyro from the driver controller.
     *
     * @return A Trigger representing the state of the start button.
     */
    public static Trigger getZeroGyro() {
        return driverController.back();
    }

    public static Trigger getResetPose() {
        return driverController.povUp();
    }

    public static Trigger snapToSpeaker(){
        return driverController.a();
    }

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static BooleanSupplier getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        driverController.start().onTrue(new InstantCommand(() -> fieldCentric =! fieldCentric));
        return () -> true;
    }

    public static Trigger getBTestButton(){
        return driverController.b();
    }

    public static Trigger getManualMode(){
    /* Changes how the angle/height of the pivot and elevator are input */
        return operatorController.back();
    }

    /* Automation */
    public static final Trigger getAutoPrepShot(){
        /* Uses a toggle switch to enable or disable automatic prep shooting */
        return operatorController.start();
    }
    public static final Trigger getAutoFire(){
        return new Trigger(() -> false);
    }
    public static final BooleanSupplier getDefense(){
        /* Uses a toggle switch to enable or disable wether we are being defended. This allows us to raise our elevator with auto shots */
        return new Trigger(() -> false);
    }

    /* Shooter */
    public static final Trigger getManualShooter(){
        return operatorController.povUp();
    }
    public static final Trigger getEjectShooter(){
        return operatorController.povDown();
    }

    /* Pivot */
    public static final DoubleSupplier getManualPivot_Output(){
        return () -> operatorController.getRightY();
    }

    /* Elevator */
    public static final DoubleSupplier getManualElevator_Output(){
        return () -> operatorController.getLeftY();
    }

    /* Elevator AND Pivot */    
    public static final Trigger setPosition_Home(){
        return operatorController.a();
    }
    
    public static final Trigger setPosition_Subwoofer(){
        return operatorController.b();
    }
    
    public static final Trigger setPosition_Amp(){
        return operatorController.y();
    }

    public static final Trigger setPosition_Trap(){
        return operatorController.x();
    }

    /* Conveyor */
    public static final Trigger getFire(){
        return operatorController.rightTrigger(TRIGGER_DEADBAND);
    }

    /* Intake */
    public static final Trigger getManualIntake(){
        return driverController.rightTrigger(TRIGGER_DEADBAND);
    }

    public static final Trigger getEjectIntake(){
        return driverController.rightBumper();
    }
}
