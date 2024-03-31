
package frc2024.dashboard.tabs;

import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.ShuffleboardConstants;
import frc2024.Constants.SwerveConstants.DriveConstants;
import frc2024.commands.swerve.FacePoint;
import frc2024.dashboard.ShuffleboardTabBase;
import frc2024.subsystems.swerve.Swerve;

public class SwerveTab extends ShuffleboardTabBase {

    private Swerve m_swerve;

    /**
     * A Shuffleboard tab for displaying and updating Swerve drive subsystem data.
     */
    public SwerveTab(Swerve swerve) {
        m_swerve = swerve;
    }
    
    //Represents a set of entries for the Swerve subsystem.
    //Each module has three entries: CANcoder, Integrated, and Velocity readings.
    private GenericEntry m_FLEncoder;
    private GenericEntry m_FLIntegrated;

    private GenericEntry m_FREncoder;
    private GenericEntry m_FRIntegrated;

    private GenericEntry m_BLEncoder;
    private GenericEntry m_BLIntegrated;

    private GenericEntry m_BREncoder;
    private GenericEntry m_BRIntegrated;

    private GenericEntry m_odometryX;
    private GenericEntry m_odometryY;
    private GenericEntry m_odometryRotation;

    private GenericEntry m_gyroYaw;

    private GenericEntry m_driveP;

    /**
     * This method creates number entries for various sensors related to the Swerve subsystem.
     * These entries are used to display and update values on the Shuffleboard.
     * Set {@code ShuffleboardConstants.UPDATE_SWERVE} to true for entries that get values.
     */
    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Swerve");

        m_FLEncoder = createNumberEntry("FL Encoder", 0, new EntryProperties(0, 0));
        m_FLIntegrated = createNumberEntry("FL Integrated", 0, new EntryProperties(1, 0));

        m_FREncoder = createNumberEntry("FR Encoder", 0, new EntryProperties(0, 1));
        m_FRIntegrated = createNumberEntry("FR Integrated", 0, new EntryProperties(1, 1));

        m_BLEncoder = createNumberEntry("BL Encoder", 0, new EntryProperties(0, 2));
        m_BLIntegrated = createNumberEntry("BL Integrated", 0, new EntryProperties(1, 2));

        m_BREncoder = createNumberEntry("BR Encoder", 0, new EntryProperties(0, 3));
        m_BRIntegrated = createNumberEntry("BR Integrated", 0, new EntryProperties(1, 3));

        m_odometryX = createNumberEntry("Odometry X", 0, new EntryProperties(2, 0));
        m_odometryY = createNumberEntry("Odometry Y", 0, new EntryProperties(2, 1));
        m_odometryRotation = createNumberEntry("Odometry Rotation", 0, new EntryProperties(2, 2));

        m_gyroYaw = createNumberEntry("Gyro Yaw", 0, new EntryProperties());

        if (ShuffleboardConstants.UPDATE_SWERVE) {
            m_driveP = createNumberEntry("Drive P Gain", DriveConstants.PID_CONSTANTS.kP(), new EntryProperties(9, 0));
        }
    }

    /**
     * Updates the values of various Shuffleboard widgets with the current state of the swerve drive.
     * Set {@code ShuffleboardConstants.UPDATE_SWERVE} to true for entries that get values.
     */
    @Override
    public void periodic() {
        m_FLEncoder.setDouble(filter(m_swerve.getModules()[0].getEncoderAngle().getDegrees()));
        m_FLIntegrated.setDouble(filter(m_swerve.getModules()[0].getPosition(true).angle.getDegrees()));

        m_FREncoder.setDouble(filter(m_swerve.getModules()[1].getEncoderAngle().getDegrees()));
        m_FRIntegrated.setDouble(filter(m_swerve.getModules()[1].getPosition(true).angle.getDegrees()));

        m_BLEncoder.setDouble(filter(m_swerve.getModules()[2].getEncoderAngle().getDegrees()));
        m_BLIntegrated.setDouble(filter(m_swerve.getModules()[2].getPosition(true).angle.getDegrees()));

        m_BREncoder.setDouble(filter(m_swerve.getModules()[3].getEncoderAngle().getDegrees()));
        m_BRIntegrated.setDouble(filter(m_swerve.getModules()[3].getPosition(true).angle.getDegrees()));

        m_odometryX.setDouble(m_swerve.getEstimatedPose().getX());
        m_odometryY.setDouble(m_swerve.getEstimatedPose().getY());
        m_odometryRotation.setDouble(m_swerve.getEstimatedHeading().getDegrees());
        
        m_gyroYaw.setDouble(m_swerve.getYaw().getDegrees());

        if (ShuffleboardConstants.UPDATE_SWERVE) {
            m_swerve.configDrivePID(DriveConstants.PID_CONSTANTS.withP(m_driveP.get().getDouble()));
        }
    }
}
