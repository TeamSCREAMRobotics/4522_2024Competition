package frc2024.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.SwerveConstants;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.swerve.Swerve;

/**
 * A command that controls the swerve drive system.
 */
public class TeleopDrive extends Command {
    private Swerve swerve;
    private DoubleSupplier[] translationSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;
    private BooleanSupplier slowModeSup;
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
    public TeleopDrive(Swerve swerve, DoubleSupplier[] translationSup, DoubleSupplier rotationSup, BooleanSupplier fieldCentricSup, BooleanSupplier slowMode) {
        addRequirements(swerve);

        this.swerve = swerve;
        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldCentricSup;
        this.slowModeSup = slowMode;
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

        boolean fieldRelative = fieldRelativeSup.getAsBoolean();
        Translation2d translationValue = 
            slowModeSup.getAsBoolean() 
            ? new Translation2d(translationSup[0].getAsDouble()*0.5, translationSup[1].getAsDouble()*0.5).times(SwerveConstants.MAX_SPEED * (fieldRelative ? AllianceFlippable.getDirectionCoefficient() : 1))
            : new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * (fieldRelative ? AllianceFlippable.getDirectionCoefficient() : 1));
        double rotationValue = rotationSup.getAsDouble()*SwerveConstants.MAX_ANGULAR_VELOCITY; //getRotation(rotationSup.getAsDouble());

        //if(Controlboard.zeroGyro().getAsBoolean()) lastAngle = AllianceFlippable.getForwardRotation();
        //FIXME causes loop overruns

        swerve.setChassisSpeeds(
            fieldRelative ? swerve.fieldRelativeSpeeds(translationValue, rotationValue) : swerve.robotRelativeSpeeds(translationValue, rotationValue),
            true
        );
    }

    /**
     * Checks if the swerve drive should start heading correction.<p>
     * If no manual input is given for a time, this method will return the rotation value required to maintain the current heading.
     * 
     * @param currentValue The current rotation value.
     * @return The determined rotation value.
     */
    private double getRotation(double currentValue){
        if(Math.abs(currentValue) > 0.1){
            correctionTimer.reset();
            return currentValue * SwerveConstants.MAX_ANGULAR_VELOCITY;
        }

        correctionTimer.start();

        if(correctionTimer.get() <= SwerveConstants.CORRECTION_TIME_THRESHOLD){
            lastAngle = swerve.getRotation();
        }

        if(correctionTimer.hasElapsed(SwerveConstants.CORRECTION_TIME_THRESHOLD)){
            return swerve.calculateHeadingCorrection(swerve.getRotation().getDegrees(), lastAngle.getDegrees());
        }

        return currentValue * SwerveConstants.MAX_ANGULAR_VELOCITY;
    }
}
