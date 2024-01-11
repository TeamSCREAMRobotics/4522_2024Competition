package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.ModuleLocation;

/**
 * A swerve drive subsystem.
 * 
 * This class provides methods for high-level control of the swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private Pigeon2 m_pigeon2;
    private SwerveModule[] m_swerveModules;
    private SwerveDriveOdometry m_odometry;
    private OdometryThread m_odometryThread;
    private ChassisSpeeds m_currentSpeeds = new ChassisSpeeds();

    /**
     * Constructs a new instance of the Swerve class.
     * 
     * Initializes the gyro, swerve modules, odometry, and auto builder.
     */
    public Swerve() {
        m_pigeon2 = new Pigeon2(Ports.PIGEON_ID, Ports.CAN_BUS_NAME);
        configGyro();
        
        /**
         * Initializes an array of SwerveModule objects with their respective names, IDs, and constants.
         * This array represents the robot's four swerve modules.
         * If there are multiple sets of modules, swap out the constants for the module in use.
         */
        m_swerveModules = new SwerveModule[] {
                new SwerveModule(ModuleLocation.FRONT_LEFT, ModuleConstants.MODULE_0),
                new SwerveModule(ModuleLocation.FRONT_RIGHT, ModuleConstants.MODULE_1),
                new SwerveModule(ModuleLocation.BACK_LEFT, ModuleConstants.MODULE_2),
                new SwerveModule(ModuleLocation.BACK_RIGHT, ModuleConstants.MODULE_3)
        };
        
        /**
         * Configures the odometry, which requires the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_odometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS, getYaw(), getModulePositions(), new Pose2d());
        m_odometryThread = new OdometryThread(m_odometry, m_swerveModules, m_pigeon2, m_swerveModules.length);
        m_odometryThread.start();

        /**
         * Configures the AutoBuilder for holonomic mode.
         * The AutoBuilder uses methods from this class to follow paths.
         */
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            this::setChassisSpeeds,
            SwerveConstants.PATH_FOLLOWER_CONFIG,
            () -> RobotContainer.getAlliance() == Alliance.Blue ? false : true,
            this
        );
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void zeroGyro() {
        m_pigeon2.setYaw(0);
    }

    /**
     * Returns a new robot-relative ChassisSpeeds based on the given inputs.
     *
     * @param translation A Translation2d representing the desired movement (m/s) in the x and y directions.
     * @param angularVel The desired angular velocity (rad/s)
     * @return The calculated ChassisSpeeds.
     */
    public ChassisSpeeds robotRelativeSpeeds(Translation2d translation, double angularVel){
        return ChassisSpeeds.discretize(translation.getX(), translation.getY(), angularVel, Constants.LOOP_TIME_SEC);
    }

    /**
     * Returns a new fie;d-relative ChassisSpeeds based on the given inputs.
     *
     * @param translation A Translation2d representing the desired movement (m/s) in the x and y directions.
     * @param angularVel The desired angular velocity (rad/s)
     * @return The calculated ChassisSpeeds.
     */
    public ChassisSpeeds fieldRelativeSpeeds(Translation2d translation, double angularVel){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), angularVel, getYaw());
        return ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME_SEC);
    }

    /**
     * Set the ChassisSpeeds to drive the robot. Use predefined methods such as {@code}robotSpeeds{@code} or create a new ChassisSpeeds object.
     * 
     * @param chassisSpeeds The ChassisSpeeds to generate states for.
     * @param isOpenLoop Whether the ChassisSpeeds is open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
        m_currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(swerveModuleStates);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    /**
     * Set the ChassisSpeeds to drive the robot. Defaults to closed loop.<p>
     * Used by AutoBuilder.
     * 
     * @param chassisSpeeds The ChassisSpeeds to generate states for.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        setChassisSpeeds(chassisSpeeds, false);
    }

    /**
     * Sets the neutral mode of the motors.<p>
     * Use {@code}NeutralModeValue.Brake{@code} or {@code}NeutralModeValue.Coast{@code}
     * 
     * @param driveMode The NeutralModeValue to set the drive motor to.
     * @param steerMode The NeutralModeValue to set the steer motor to.
     */
    public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode){
        for (SwerveModule mod : m_swerveModules) {
            mod.setDriveNeutralMode(driveMode);
            mod.setSteerNeutralMode(steerMode);
        }
    }

    /** 
     * Calculates the hold value based on the provided current and last angles.
     *  
     * @param currentAngle The current angle measurement.
     * @param lastAngle The desired angle to calculate towards.
     * @return The calculated value from the heading controller.
    */
    public double calculateHeadingCorrection(double currentAngle, double lastAngle){
        return SwerveConstants.HEADING_CONSTANTS.toPIDController().calculate(currentAngle, lastAngle);
    }
    
    /**
     * Resets the pose reported by the odometry to the specified pose.
     *
     * @param pose The new pose to set.
     */
    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Retrieves the estimated pose of the odometry.
     *
     * @return The current pose of the odometry.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns an array composed of the swerve modules in the system.
     *
     * @return The array of SwerveModule objects.
     */
    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    /**
     * Returns an array composed of the state of each module in the system.
     *
     * @return The array of SwerveModuleState objects.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveModules) {
            states[mod.getModuleNumber()] = mod.getState(true);
        }
        return states;
    }

    /**
     * Returns an array composed of the positions of each module in the system.
     *
     * @return The array of SwerveModulePosition objects.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveModules) {
            positions[mod.getModuleNumber()] = mod.getPosition(true);
        }
        return positions;
    }

    /**
     * Returns the current robot-centric ChassisSpeeds.<p>
     * Used by AutoBuilder.
     * 
     * @return The current robot-centric ChassisSpeeds.
     */
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(m_currentSpeeds, getYaw());
    }

    /**
     * Returns the yaw rotation in degrees.
     * If {@code}invertGyro{@code} is set to true, the yaw rotation is inverted.
     *
     * @return The yaw rotation as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return (SwerveConstants.GYRO_INVERT) ? m_pigeon2.getRotation2d().minus(Rotation2d.fromDegrees(360))
                : m_pigeon2.getRotation2d();
    }

    /**
     * Returns the gyro object.<p>
     * Used by SwerveTab to display to Shuffleboard.
     * 
     * @return The gyro object.
     */
    public Pigeon2 getGyro() {
        return m_pigeon2;
    }

    /**
     * Configures the gyro.
     * See DeviceConfig for more information.
     */
    public void configGyro() {
        DeviceConfig.configurePigeon2("Swerve Pigeon", m_pigeon2, DeviceConfig.swervePigeonConfig(), Constants.LOOP_TIME_HZ);
    }

   /**
     * Configures all module drive motors with the given constants.
     * 
     * @param constants ScreamPIDConstants to be applied.
     */
    public void configDrivePID(ScreamPIDConstants constants){
        for (SwerveModule mod : m_swerveModules) {
            mod.configDriveMotorPID(constants);
        }
    }

    /**
     * Called periodically through SubsystemBase
     */
    @Override
    public void periodic() {}
}