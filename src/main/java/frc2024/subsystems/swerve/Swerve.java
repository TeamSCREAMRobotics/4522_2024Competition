package frc2024.subsystems.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.Ports;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.VisionConstants;
import frc2024.Constants.SwerveConstants.ModuleConstants;
import frc2024.Constants.SwerveConstants.ModuleConstants.ModuleLocation;
import frc2024.subsystems.Vision;
import frc2024.subsystems.Vision.Limelight;

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
    private SwerveDriveOdometry m_odometry;
    //private PoseEstimator m_poseEstimator;
    private OdometryThread m_odometryThread;
    private ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] m_desiredStates = new SwerveModuleState[4];

    private PIDController m_headingController = SwerveConstants.HEADING_CONSTANTS.toPIDController();
    private PIDController m_snapController = SwerveConstants.SNAP_CONSTANTS.toPIDController();

    private SlewRateLimiter m_snapFilter = new SlewRateLimiter(60);

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
            SwerveConstants.KINEMATICS, 
            getYaw(), 
            getModulePositions(), 
            new Pose2d(), 
            VisionConstants.STATE_STD_DEVS,
            VisionConstants.VISION_STD_DEVS
        );

        m_odometry = new SwerveDriveOdometry(
            SwerveConstants.KINEMATICS, 
            getYaw(), 
            getModulePositions()
        );
        
        m_odometryThread = new OdometryThread();
        m_odometryThread.start();

        m_headingController.enableContinuousInput(-180.0, 180.0);
        m_snapController.enableContinuousInput(-180.0, 180.0);
    }

    /**
     * Configures the AutoBuilder for holonomic mode.
     * The AutoBuilder uses methods from this class to follow paths.
     */
    public void configureAutoBuilder(){
        AutoBuilder.configureHolonomic(
            this::getOdometryPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            this::setChassisSpeeds,
            SwerveConstants.PATH_FOLLOWER_CONFIG,
            () -> {
                return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
            },
            this
        );
    }

    public void stopOdometryThread(){
        m_odometryThread.stop();
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void resetYaw(Rotation2d rotation) {
        m_pigeon2.setYaw(rotation.getDegrees());
    }

    public void resetEstimatedHeading(Rotation2d rotation) {
        m_poseEstimator.resetPosition(getYaw(), getModulePositions(), new Pose2d(getEstimatedPose().getTranslation(), rotation));
    }

    public void resetOdometryHeading(Rotation2d rotation) {
        m_poseEstimator.resetPosition(getYaw(), getModulePositions(), new Pose2d(getOdometryPose().getTranslation(), rotation));
    }

    public void resetOdometryPose(Pose2d pose){
        m_odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public Command resetOdometryToEstimated(){
        return Commands.runOnce(() -> resetOdometryPose(m_poseEstimator.getEstimatedPosition()));
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
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), angularVel, getEstimatedHeading());
        return ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME_SEC);
    }

    public ChassisSpeeds snappedFieldRelativeSpeeds(Translation2d translation, Rotation2d angle, Rotation2d angleThreshold){
        return fieldRelativeSpeeds(translation, calculateSnapOutput(angle, angleThreshold));
    }

    /**
     * Set the ChassisSpeeds to drive the robot. Use predefined methods such as {@code}robotSpeeds{@code} or create a new ChassisSpeeds object.
     * 
     * 
     * @param chassisSpeeds The ChassisSpeeds to generate states for.
     * @param isOpenLoop Whether the ChassisSpeeds is open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters, boolean isOpenLoop){
        m_desiredSpeeds = chassisSpeeds;
        m_desiredStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(m_desiredSpeeds, centerOfRotationMeters);

        SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredStates, SwerveConstants.MAX_SPEED);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(m_desiredStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop){
        setChassisSpeeds(chassisSpeeds, new Translation2d(), isOpenLoop);
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

    public void overrideRotation(double omegaRadiansPerSecond){
        m_desiredSpeeds.omegaRadiansPerSecond = omegaRadiansPerSecond;
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

    public double calculateSnapOutput(Rotation2d targetAngle, Rotation2d angleThreshold){
        m_snapController.setTolerance(angleThreshold.getDegrees());
        double output = m_snapFilter.calculate(m_snapController.calculate(getEstimatedHeading().getDegrees(), targetAngle.getDegrees()));
        if(m_snapController.atSetpoint()){
            return 0.0;
        } else {
            return output;
        }
    }

    public boolean snapAtSetpoint(){
        return m_snapController.atSetpoint();
    }

    public boolean atAngleThreshold(Rotation2d target, Rotation2d threshold){
        return Math.abs(target.minus(getEstimatedHeading()).getDegrees()) < threshold.getDegrees();
    }
    
    /**
     * Resets the pose reported by the odometry to the specified pose.
     *
     * @param pose The new pose to set.
     */
    public void resetPose(Pose2d pose) {
        resetModulePositions();
        m_poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
        m_odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetPose_Apriltag(){
        if(Vision.getBotPose2d(Limelight.SHOOT_SIDE).getX() != 0.0){
            Translation2d translation = Vision.getBotPose2d(Limelight.SHOOT_SIDE).getTranslation();
            m_poseEstimator.resetPosition(getYaw(), getModulePositions(), new Pose2d(translation, getEstimatedHeading()));
        }
    }

    /**
     * Retrieves the estimated pose of the odometry.
     *
     * @return The current pose of the odometry.
     */
    public Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose(){
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

    public void resetModulePositions(){
        for(SwerveModule mod : m_swerveModules) {
            mod.resetModulePosition();
        }
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

    public ChassisSpeeds getFieldRelativeSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getYaw());
    }

    public double getRobotSpeed(){
        return Math.sqrt(Math.pow(getRobotRelativeSpeeds().vxMetersPerSecond, 2) + Math.pow(getRobotRelativeSpeeds().vyMetersPerSecond, 2));
    }

    /**
     * Returns the odometry rotation in degrees.
     *
     * @return The odometry rotation as a Rotation2d.
     */
    public Rotation2d getEstimatedHeading() {
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getOdometryHeading() {
        return m_odometry.getPoseMeters().getRotation();
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
        if(getModulePositions() != null){
            m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        }
        Vision.updateEstimateWithValidMeasurements(Limelight.SHOOT_SIDE, m_poseEstimator);
    }

    public void logOutputs(){
        Logger.recordOutput("RobotState/OdometryPose", getOdometryPose());
        Logger.recordOutput("RobotState/EstimatedPose", getEstimatedPose());
        Logger.recordOutput("RobotState/GyroAngle", getYaw());
        Logger.recordOutput("Swerve/Setpoint/DesiredSpeeds", m_desiredSpeeds);
        Logger.recordOutput("Swerve/Measured/FieldSpeeds", getFieldRelativeSpeeds());
        Logger.recordOutput("Swerve/Measured/RobotSpeeds", getRobotRelativeSpeeds());
        Logger.recordOutput("Swerve/Measured/States", getModuleStates());
        if(m_desiredStates[0] != null){
            Logger.recordOutput("Swerve/Setpoint/DesiredStates", m_desiredStates);
        }
        Logger.recordOutput("Swerve/Measured/ModulePositions", getModulePositions());
        if(getCurrentCommand() != null){
            Logger.recordOutput("Swerve/CurrentCommand", getCurrentCommand().getName());
        }
        for(SwerveModule mod : getModules()){
            mod.logOutputs();
        }
    }

    public Command resetPoseCommand(Pose2d pose){
        return Commands.runOnce(() -> resetPose(pose));
    }

    public Command overrideRotationTargetCommand(Supplier<Rotation2d> rotation){
        Command command;
        if(rotation == null){ 
            command = Commands.run(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()));
        } else {
            command = Commands.run(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(rotation.get())));
        }
        return command;
    }

    public Command driveCommand(ChassisSpeeds speeds, boolean fieldRelative){
        return run(() -> setChassisSpeeds(speeds, new Translation2d(), fieldRelative));
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

                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
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

