package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlippable;
import frc.robot.Constants.SwerveConstants;
import frc.robot.controlboard.Controlboard;
import frc.robot.subsystems.swerve.Swerve;

/**
 * A command that controls the swerve drive system.
 */
public class TeleopSwerve extends Command {
    private Swerve swerve;
    private Supplier<Translation2d> translationSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;
    private Rotation2d lastAngle;
    private Timer correctionTimer = new Timer();

    /**
     * Constructs a TeleopSwerve command with the given parameters.
     *
     * @param swerve The Swerve subsystem to control.
     * @param translationSup A supplier for the translation value.
     * @param strafeSup A supplier for the strafe value.
     * @param rotationSup A supplier for the rotation value.
     * @param fieldCentricSup A supplier for the drive mode. Robot centric = false; Field centric = true
     */
    public TeleopSwerve(Swerve swerve, Supplier<Translation2d> translationSup, DoubleSupplier rotationSup, BooleanSupplier fieldCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldCentricSup;
    }

    @Override
    public void initialize() {
        correctionTimer.stop();
        correctionTimer.reset();
        lastAngle = swerve.getRotation();
    } 

    /**
     * Executes the swerve drive command.
     * <p>This method applies a deadband to the translation, strafe, and rotation values
     * and then passes them to the swerve drive subsystem to drive the robot.
     */
    @Override
    public void execute() {
        
        Translation2d translationValue = translationSup.get().times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
        System.out.println(translationValue);
        double rotationValue = getRotation(rotationSup.getAsDouble());
        boolean fieldRelative = fieldRelativeSup.getAsBoolean();

        if(Controlboard.getZeroGyro().getAsBoolean()) lastAngle = AllianceFlippable.getForwardRotation();

        swerve.setChassisSpeeds(
            fieldRelative ? swerve.fieldRelativeSpeeds(translationValue, rotationValue) : swerve.robotRelativeSpeeds(translationValue, rotationValue),
            true
        );
    }

    /**
     * Checks if the swerve drive should start heading correction.<p>
     * If no manual input is given for a time, this method will return the rotation value required to maintain the current heading.
     * 
     * @param current The current rotation value.
     * @return The determined rotation value.
     */
    private double getRotation(double current){
        if(Math.abs(current) > 0.1){
            correctionTimer.reset();
            return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
        }

        correctionTimer.start();

        if(correctionTimer.get() <= SwerveConstants.CORRECTION_TIME_THRESHOLD){
            lastAngle = swerve.getRotation();
        }

        if(correctionTimer.hasElapsed(SwerveConstants.CORRECTION_TIME_THRESHOLD)){
            return swerve.calculateHeadingCorrection(swerve.getRotation().getDegrees(), lastAngle.getDegrees());
        }

        return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
    }
}
