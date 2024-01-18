package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
    private DoubleSupplier[] translation;
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
    public TeleopSwerve(Swerve swerve, DoubleSupplier[] translation, DoubleSupplier rotationSup, BooleanSupplier fieldCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translation = translation;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldCentricSup;
    }

    @Override
    public void initialize() {
        correctionTimer.stop();
        correctionTimer.reset();
        lastAngle = swerve.getYaw();
    } 

    /**
     * Executes the swerve drive command.
     * <p>This method applies a deadband to the translation, strafe, and rotation values
     * and then passes them to the swerve drive subsystem to drive the robot.
     */
    @Override
    public void execute() {
        
        Translation2d translationVal = new Translation2d(translation[0].getAsDouble(), translation[1].getAsDouble()).times(SwerveConstants.MAX_SPEED).times(AllianceFlippable.getDirectionCoefficient());
        double rotationVal = getRotation(rotationSup.getAsDouble());
        boolean fieldRelativeVal = fieldRelativeSup.getAsBoolean();

        if(Controlboard.getZeroGyro().getAsBoolean()) lastAngle = AllianceFlippable.getForwardRotation();

        swerve.setChassisSpeeds(
            fieldRelativeVal ? swerve.fieldRelativeSpeeds(translationVal, rotationVal) : swerve.robotRelativeSpeeds(translationVal, rotationVal),
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
        boolean rotating = Math.abs(current) > 0.1;

        if(rotating){
            correctionTimer.reset();
            return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
        }

        correctionTimer.start();

        if(correctionTimer.get() <= SwerveConstants.CORRECTION_TIME_THRESHOLD){
            lastAngle = swerve.getYaw();
        }

        if(correctionTimer.hasElapsed(SwerveConstants.CORRECTION_TIME_THRESHOLD)){
            return swerve.calculateHeadingCorrection(swerve.getYaw().getDegrees(), lastAngle.getDegrees());
        }

        return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
    }
}
