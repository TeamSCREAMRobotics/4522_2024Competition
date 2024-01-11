package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometryThread extends Thread {
    private BaseStatusSignal[] m_allSignals;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;
    public int ModuleCount;

    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;
    private double averageLoopTime = 0;

    private SwerveModule[] m_modules;
    private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];
    private SwerveDriveOdometry m_odometry;
    private Pigeon2 m_pigeon2;

    public OdometryThread(SwerveDriveOdometry odometry, SwerveModule[] modules, Pigeon2 pigeon2, int modCount) {
        super();
        // 4 signals for each module + 2 for Pigeon2
        ModuleCount = modCount;
        m_modules = modules;
        m_odometry = odometry;
        m_pigeon2 = pigeon2;
        m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];

        for (int i = 0; i < ModuleCount; ++i) {
            var signals = m_modules[i].getSignals();
            m_allSignals[(i * 4) + 0] = signals[0];
            m_allSignals[(i * 4) + 1] = signals[1];
            m_allSignals[(i * 4) + 2] = signals[2];
            m_allSignals[(i * 4) + 3] = signals[3];
        }
        m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
        m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZWorld();
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
                m_modulePositions[i] = m_modules[i].getPosition(false);
            }

            Rotation2d yawDegrees = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZWorld()));

            m_odometry.update(yawDegrees, m_modulePositions);
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