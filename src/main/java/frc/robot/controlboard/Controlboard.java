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
        return operatorController.back();
    }

    /* Shooter */
    // public static final Trigger getManualPrepFire(){
    //     return operatorController.start(); //For subwoofer/amp/trap shots
    // }

    public static final Trigger getEjectShooter(){
        return operatorController.leftTrigger(TRIGGER_DEADBAND);
    }

    public static final Trigger getAutoPrepShot(){
        return operatorController.leftBumper();
    }

    /* Pivot */
    public static final DoubleSupplier getManualPivot_Output(){
        return () -> operatorController.getRightY();
    }
    
    public static final Trigger setPivotPosition_Home(){
        return operatorController.povDown();
    }
    
    public static final Trigger setPivotPosition_Subwoofer(){
        return operatorController.povRight();
    }
    
    public static final Trigger setPivotPosition_Amp(){
        return operatorController.povUp();
    }

    public static final Trigger setPivotPosition_Trap(){
        return operatorController.povLeft();
    }

    /* Elevator */
    public static final DoubleSupplier getManualElevator_Output(){
        return () -> operatorController.getLeftY();
    }
    
    public static final Trigger setElevatorPosition_Home(){
        return operatorController.a();
    }
    
    public static final Trigger setElevatorPosition_Subwoofer(){
        return operatorController.b();
    }
    
    public static final Trigger setElevatorPosition_Amp(){
        return operatorController.y();
    }

    public static final Trigger setElevatorPosition_Trap(){
        return operatorController.x();
    }

    /* Conveyor */
    public static final Trigger getFire(){
        return operatorController.start();
    }

    /* Intake */
    public static final Trigger getManualIntake(){
        return driverController.rightTrigger(TRIGGER_DEADBAND);
    }

    public static final Trigger getEjectIntake(){
        return driverController.rightBumper();
    }
}
