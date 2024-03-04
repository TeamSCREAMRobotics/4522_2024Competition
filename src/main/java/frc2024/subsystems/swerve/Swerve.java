package frc2024.subsystems.swerve;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.AllianceFlippable;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.Ports;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.Constants.SwerveConstants.ModuleConstants;
import frc2024.Constants.SwerveConstants.ModuleConstants.ModuleLocation;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;
import frc2024.subsystems.Vision.TimestampedVisionMeasurement;

/**
 * A swerve drive subsystem.
 * 
 * This class provides methods for high-level control of the swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private Pigeon2 m_pigeon2;
    private SwerveModule[] m_swerveModules;
    private SwerveModulePosition[] m_modulePositions;
    private SwerveDrivePoseEstimator m_poseEstimator;
    //private PoseEstimator m_poseEstimator;
    private OdometryThread m_odometryThread;
    private ChassisSpeeds m_currentSpeeds = new ChassisSpeeds();

    private PIDController m_headingController = SwerveConstants.HEADING_CONSTANTS.toPIDController();
    private PIDController m_snapController = SwerveConstants.SNAP_CONSTANTS.toPIDController();

    /**
     * Constructs a new instance of the Swerve class.
     * 
     * Initializes the gyro, swerve modules, odometry, and auto builder.
     */
    public Swerve() {
        m_pigeon2 = new Pigeon2(Ports.PIGEON_ID, Ports.CANIVORE_NAME);
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

        m_modulePositions = new SwerveModulePosition[4];
        
        /**
         * Configures the pose estimator, which requires the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.POSE_ESTIMATOR_KINEMATICS, 
            getYaw(), 
            getModulePositions(), 
            new Pose2d(), 
            VisionConstants.STATE_STD_DEVS,
            VisionConstants.VISION_STD_DEVS
        );
        
        m_odometryThread = new OdometryThread();
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
            () -> {
                return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
            },
            this
        );

        m_headingController.enableContinuousInput(-180, 180);
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void resetYaw(Rotation2d rotation) {
        m_pigeon2.setYaw(rotation.getDegrees());
        m_poseEstimator.resetPosition(rotation, getModulePositions(), getPose());
    }

    /**
     * Returns a new robot-relative ChassisSpeeds based on the given inputs.
     *
     * @param translation A Translation2d representing the desired movement (m/s) in the x and y directions.
     * @param angularVel The desired angular velocity (rad/s)
     * @return The calculated ChassisSpeeds.
     */
    public ChassisSpeeds robotRelativeSpeeds(Translation2d translation, double angularVel){
        return new ChassisSpeeds(translation.getX(), translation.getY(), angularVel);
    }

    /**
     * Returns a new fie;d-relative ChassisSpeeds based on the given inputs.
     *
     * @param translation A Translation2d representing the desired movement (m/s) in the x and y directions.
     * @param angularVel The desired angular velocity (rad/s)
     * @return The calculated ChassisSpeeds.
     */
    public ChassisSpeeds fieldRelativeSpeeds(Translation2d translation, double angularVel){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), angularVel, getRotation());
        return ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME_SEC);
    }

    public ChassisSpeeds snappedFieldRelativeSpeeds(Translation2d translation, Rotation2d angle){
        return fieldRelativeSpeeds(translation, m_snapController.calculate(getRotation().getDegrees(), angle.getDegrees()));
    }

    /**
     * Set the ChassisSpeeds to drive the robot. Use predefined methods such as {@code}robotSpeeds{@code} or create a new ChassisSpeeds object.
     * 
     * @param chassisSpeeds The ChassisSpeeds to generate states for.
     * @param isOpenLoop Whether the ChassisSpeeds is open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop){
        m_currentSpeeds = chassisSpeeds;
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
        return m_headingController.calculate(currentAngle, lastAngle);
    }
    
    /**
     * Resets the pose reported by the odometry to the specified pose.
     *
     * @param pose The new pose to set.
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
     * Retrieves the estimated pose of the odometry.
     *
     * @return The current pose of the odometry.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
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
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the odometry rotation in degrees.
     *
     * @return The odometry rotation as a Rotation2d.
     */
    public Rotation2d getRotation() {
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getYaw(){
        return (SwerveConstants.GYRO_INVERT) ? m_pigeon2.getRotation2d().minus(Rotation2d.fromDegrees(360))
                :  m_pigeon2.getRotation2d();
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
        m_pigeon2.getAngularVelocityZWorld().setUpdateFrequency(100.0);
        m_pigeon2.optimizeBusUtilization();
        //resetYaw(AllianceFlippable.getForwardRotation());
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

    public void stopAll(){
        for(SwerveModule mod : m_swerveModules){
            mod.stopAll();
        }
    }

    /**
     * Called periodically through SubsystemBase
     */
    @Override
    public void periodic() {
        if(Vision.getTimestampedVisionMeasurement(Limelight.SHOOTER).isPresent()){
            TimestampedVisionMeasurement measurement = Vision.getTimestampedVisionMeasurement(Limelight.SHOOTER).get();
            m_poseEstimator.addVisionMeasurement(measurement.pose(), measurement.timestamp());
        }
    }

    public Command resetPoseCommand(Pose2d pose){
        return Commands.runOnce(() -> resetPose(pose));
    }

    public Command overrideRotationTargetCommand(Rotation2d rotation){
        Command command;
        if(rotation == null){ 
            command = Commands.runOnce(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
        } else {
            command = Commands.runOnce(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(rotation)));
        }
        return command;
    }

    public Command driveCommand(ChassisSpeeds speeds, boolean fieldRelative){
        return run(() -> setChassisSpeeds(speeds, fieldRelative));
    }

    private class OdometryThread extends Thread {
        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;
        public int ModuleCount = m_swerveModules.length;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                var signals = m_swerveModules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZDevice();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : m_allSignals) {
                sig.setUpdateFrequency(250);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);

                /* Get status of the waitForAll */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < ModuleCount; ++i) {
                    /* No need to refresh since it's automatically refreshed from the waitForAll() */
                    m_modulePositions[i] = m_swerveModules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees =
                        BaseStatusSignal.getLatencyCompensatedValue(
                                m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZDevice());

                m_poseEstimator.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
            }
        }
        
        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDaqs() {
            return SuccessfulDaqs;
        }

        public int getFailedDaqs() {
            return FailedDaqs;
        }
    }
}

