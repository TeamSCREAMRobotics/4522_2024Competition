package frc2024.commands.swerve;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team4522.lib.util.AllianceFlippable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.SwerveConstants;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.swerve.Swerve;

/**
 * A command that controls the swerve drive system.
 */
public class SnappedDrive extends Command {
    private Swerve swerve;
    private DoubleSupplier[] translationSup;
    private BooleanSupplier slowModeSup;
    private DoubleSupplier snapAngleSup;

    /**
     * Constructs a TeleopSwerve command with the given parameters.
     *
     * @param swerve The Swerve subsystem to control.
     * @param translationSup A supplier for the translation value.
     * @param strafeSup A supplier for the strafe value.
     * @param rotationSup A supplier for the rotation value.
     * @param fieldCentricSup A supplier for the drive mode. Robot centric = false; Field centric = true
     */
    public SnappedDrive(Swerve swerve, DoubleSupplier[] translationSup, DoubleSupplier snapAngle, BooleanSupplier slowMode) {
        addRequirements(swerve);

        this.swerve = swerve;
        this.translationSup = translationSup;
        this.slowModeSup = slowMode;
        this.snapAngleSup = snapAngle;
    }

    public SnappedDrive(Swerve mSwerve, DoubleSupplier[] translation, Supplier<OptionalDouble> supplier,
            BooleanSupplier booleanSupplier, Object object) {
        //TODO Auto-generated constructor stub
    }

    @Override
    public void initialize() {} 

    /**
     * Executes the swerve drive command.
     * <p>This method applies a deadband to the translation, strafe, and rotation values
     * and then passes them to the swerve drive subsystem to drive the robot.
     */
    @Override
    public void execute() {
        Translation2d translationValue = 
            slowModeSup.getAsBoolean() 
            ? new Translation2d(translationSup[0].getAsDouble()*0.5, translationSup[1].getAsDouble()*0.5).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient())
            : new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlippable.getDirectionCoefficient());
        Rotation2d snapAngle = Rotation2d.fromDegrees(snapAngleSup.getAsDouble());
        
        ChassisSpeeds targetSpeeds;
        targetSpeeds = swerve.snappedFieldRelativeSpeeds(translationValue, snapAngle);
        
        swerve.setChassisSpeeds(targetSpeeds, true);
    }
}
