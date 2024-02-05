package frc2024.controlboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2024.Constants.SwerveConstants;

/**
 * A utility class that contains button bindings.
 * 
 * Controlboard allows easy reference of custom button associations.
 */
public class Controlboard{

    public static final double STICK_DEADBAND = 0.05;
    public static final double TRIGGER_DEADBAND = 0.10;
    public static final Rotation2d SNAP_TO_POLE_THRESHOLD = Rotation2d.fromDegrees(7.0);

    private static final CommandXboxController driverController = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    private static boolean fieldCentric = true;

    /**
     * Retrieves the swerve translation from the driver controller.
     * 
     * @return A DoubleSupplier array representing the x and y values from the controller.
     */
    public static DoubleSupplier[] getTranslation(){
        double x = -MathUtil.applyDeadband(driverController.getLeftY(), STICK_DEADBAND);
        double y = -MathUtil.applyDeadband(driverController.getLeftX(), STICK_DEADBAND);
        /* if(getAutoFire().getAsBoolean()){
            return new DoubleSupplier[] {
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), STICK_DEADBAND)*SwerveConstants.SHOOT_WHILE_MOVING_SCALAR,
                () -> -MathUtil.applyDeadband(driverController.getLeftX(), STICK_DEADBAND)*SwerveConstants.SHOOT_WHILE_MOVING_SCALAR
            };
        } */ // TODO Find a better way to do this
        
        return new DoubleSupplier[] {
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), STICK_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), STICK_DEADBAND)
        };
    }

    private static Translation2d snapTranslationToPole(Translation2d translation){
		for(int i = 0; i < 360; i+=90){
			if(Math.abs(translation.getAngle().minus(Rotation2d.fromDegrees(i)).getDegrees()) < SNAP_TO_POLE_THRESHOLD.getDegrees()){
                return new Translation2d(translation.getNorm(), Rotation2d.fromDegrees(i));
            }
		}
        return translation;
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
        driverController.start().onTrue(Commands.runOnce(() -> fieldCentric =! fieldCentric));
        return () -> fieldCentric;
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
        /* Uses a toggle switch to enable or disable automatic firing when requirements are met */
        return new Trigger(() -> false);
    }
    public static final Trigger getDefense(){
        /* Uses a toggle switch to enable or disable wether we are being defended. This allows us to raise our elevator with auto shots */
        return new Trigger(() -> false);
    }
    public static final Trigger getAutoclimb(){
        return new Trigger(() -> false);
    }

    /* Climber */
    public static final Trigger getManualClimber_Up(){
        return new Trigger(() -> false); //Up POV on the button board
    }
    public static final Trigger getManualClimber_Down(){
        return new Trigger(() -> false); //Down POV on the button board
    }

    /* Shooter */
    public static final Trigger getManualShooter(){
        return driverController.povUp();
    }
    public static final Trigger getEjectShooter(){
        return driverController.povDown();
    }

    /* Pivot */
    public static final DoubleSupplier getManualPivot_Output(){
        return () -> operatorController.getRightY();
    }

    /* Elevator */
    public static final DoubleSupplier getManualElevator_Output(){
        return () -> operatorController.getLeftY()/2;
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

    public static final Trigger setPosition_Trap_Floor(){
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

    public static final Trigger getAutoPickup(){
        return driverController.leftBumper();
    }
}