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

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static BooleanSupplier getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        driverController.start().onTrue(new InstantCommand(() -> fieldCentric =! fieldCentric));
        return () -> fieldCentric;
    }

    public static Trigger getBTestButton(){
        return driverController.b();
    }

    /* Shooter */
    public static final Trigger getManualFire(){
        return operatorController.rightTrigger(0.10);
    }

    public static Trigger getPrepShot(){
        return operatorController.rightBumper();
    }

    public static Trigger getEject(){ //TODO should eject run an ejection through the shooter flywheel or the intake? Either?
        return operatorController.leftTrigger(0.10);
    }

    /* Pivot */
    public static Trigger getManualPivot_Boolean(){
        return operatorController.back();
    }
    public static DoubleSupplier getManualPivot_Output(){
        return () -> operatorController.getRightY();
    }
    //TODO Create setpoint buttons

    /* Elevator */
    public static Trigger getManualElevator_Boolean(){
        return operatorController.start();
    }
    public static DoubleSupplier getManualElevator_Output(){
        return () -> operatorController.getLeftY();
    }
    //TODO Create setpoint buttons

    /* Conveyor */
    public static final Trigger getAutoFire(){
        return operatorController.a();
    }
}
