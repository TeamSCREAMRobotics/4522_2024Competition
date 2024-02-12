package frc2024.controlboard;

import java.util.DoubleSummaryStatistics;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
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

    private static final CommandXboxController driverController_Command = new CommandXboxController(0);
    private static final XboxController driverController = new XboxController(0);
    private static final CommandXboxController operatorController_Command = new CommandXboxController(1);
    private static final XboxController operatorController = new XboxController(1);
    private static final Buttonboard buttonBoard = new Buttonboard(2, 3);

    private static boolean fieldCentric = true;

    /**
     * Retrieves the swerve translation from the driver controller.
     * 
     * @return A DoubleSupplier array representing the x and y values from the controller.
     */
    public static DoubleSupplier[] getTranslation(){
        double x = -MathUtil.applyDeadband(driverController_Command.getLeftY(), STICK_DEADBAND);
        double y = -MathUtil.applyDeadband(driverController_Command.getLeftX(), STICK_DEADBAND);
        return snapTranslationToPole(
            new DoubleSupplier[]{
                () -> x,
                () -> y
            });
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

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return A DoubleSupplier representing the rotation.
     */
    public static DoubleSupplier getRotation() {
        return () -> -MathUtil.applyDeadband(driverController_Command.getRightX(), STICK_DEADBAND);
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

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static BooleanSupplier getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        driverController_Command.start().onTrue(Commands.runOnce(() -> fieldCentric =! fieldCentric));
        return () -> fieldCentric;
    }

    public static Trigger test(){
        return driverController_Command.b();
    }

    public static Trigger manualMode(){
    /* Changes how the angle/height of the pivot and elevator are input */
        return driverController_Command.povRight();
    }

    /* Automation */
    public static final Trigger prepShot(){
        /* Uses a toggle switch to enable or disable automatic prep shooting */
        return operatorController_Command.start();
    }

    public static final Trigger autoFire(){
        /* Uses a toggle switch to enable or disable automatic firing when requirements are met */
        return new Trigger(() -> false);
    }

    public static final BooleanSupplier defendedMode(){
        /* Uses a toggle switch to enable or disable wether we are being defended. This allows us to raise our elevator with auto shots */
        return () -> false;
    }

    public static final Trigger autoClimb(){
        return new Trigger(() -> false);
    }

    public static Trigger snapToSpeaker(){
        return driverController_Command.a();
    }

    /* Climber */
    public static final DoubleSupplier getManualClimberOutput(){
        return () -> (driverController_Command.getLeftTriggerAxis() - driverController_Command.getRightTriggerAxis())/5; //Up POV on the button board
    }

    /* Shooter */
    public static final Trigger manuallyShoot(){
        return driverController_Command.povUp();
    }
    public static final Trigger ejectThroughShooter(){
        return driverController_Command.povDown();
    }

    /* Pivot */
    public static final DoubleSupplier getManualPivotOutput(){
        return () -> operatorController_Command.getRightY();
    }

    /* Elevator */
    public static final DoubleSupplier getManualElevatorOutput(){
        return () -> driverController_Command.getLeftY()/2;
    }

    /* Elevator AND Pivot */    
    public static final Trigger goToHomePosition(){
        return operatorController_Command.a();
    }
    
    public static final Trigger goToSubwooferPosition(){
        return operatorController_Command.b();
    }
    
    public static final Trigger goToAmpPosition(){
        return operatorController_Command.y();
    }

    /* Intake */
    public static final Trigger intakeFromFloor(){
        return driverController_Command.rightTrigger(TRIGGER_DEADBAND);
    }

    public static final Trigger ejectThroughIntake(){
        return driverController_Command.rightBumper();
    }

    public static final Trigger autoPickupFromFloor(){
        return driverController_Command.leftBumper();
    }
}
