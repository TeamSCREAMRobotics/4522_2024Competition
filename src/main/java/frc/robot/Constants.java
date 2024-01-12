package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.COTSFalconSwerveConstants;

/**
 * A class for constants used in various places in the project.
 */
public final class Constants{

    public record MotionMagicConstants(double cruiseVelocity, double acceleration, int sCurveStrength){}

    /* Robot loop time */
    public static final double LOOP_TIME_SEC = 0.02;
    public static final double LOOP_TIME_HZ = 1 / LOOP_TIME_SEC;

    public static final class Ports {
        /** Possible CAN bus strings are:
         *   • "rio" for the native roboRIO CAN bus 
         *   • CANivore name or serial number 
         *   • "*" for any CANivore seen by the program
         */
        public static final String CAN_BUS_NAME = "canivore"; // TODO ROBOT SPECIFIC

        /* Pigeon2 */
        public static final int PIGEON_ID = 0; // TODO ROBOT SPECIFIC
    }

    
    public static final class ShuffleboardConstants {

        /* For updating values like PID from Shuffleboard */
        public static final boolean UPDATE_SWERVE = false;
    }


    public static final class SwerveConstants {

        /* Drivebase Constants */
        // TODO ROBOT SPECIFIC
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.75); // Distance from left wheels to right wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(22.65); // Distance from front wheels to back wheels

        /* Gyro Constants */
        public static final boolean GYRO_INVERT = false; // TODO Always ensure gyro reads CCW+ CW-

        /* Swerve Kinematics */
        public static final double MAX_SPEED = 5.0; // m/s theoretical = 5.7
        public static final double MAX_ACCELERATION = 4.0; // m/s^2 theoretical
        public static final double MAX_ANGULAR_VELOCITY = 8.0; // rad/s

        /* Swerve Kinematics */
        // No need to ever change this unless there are more than four modules.
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /* Selected Module Constants */
        // TODO ROBOT SPECIFIC
        public static final COTSFalconSwerveConstants MODULE_TYPE = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.L3); 

        /* Swerve Heading Correction */
        public static final ScreamPIDConstants HEADING_CONSTANTS = new ScreamPIDConstants(0.1, 0.0, 0.001);
        public static final double CORRECTION_TIME_THRESHOLD = 0.2;

        /* PathPlanner Constants */
        public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS = new ScreamPIDConstants(100, 0.0, 0.0); // TODO ROBOT SPECIFIC
        public static final ScreamPIDConstants PATH_ROTATION_CONSTANTS = new ScreamPIDConstants(20, 0.0, 0.0);

        public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                PATH_TRANSLATION_CONSTANTS.toPathPlannerPIDConstants(), 
                PATH_ROTATION_CONSTANTS.toPathPlannerPIDConstants(), 
                MAX_SPEED, 
                new Translation2d(SwerveConstants.WHEEL_BASE/2, SwerveConstants.TRACK_WIDTH/2).getNorm(), 
                new ReplanningConfig(),
                LOOP_TIME_SEC
        );

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            MAX_ANGULAR_VELOCITY, 
            CORRECTION_TIME_THRESHOLD, 
            LOOP_TIME_SEC, 
            LOOP_TIME_HZ
        );

        
        public static final class DriveConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = MODULE_TYPE.driveGearRatio;

            /* Neutral Mode */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = MODULE_TYPE.driveMotorInvert;

            /* Current Limit Constants */
            public static final int SUPPLY_CURRENT_LIMIT = 35;
            public static final int SUPPLY_CURRENT_THRESHOLD = 60;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;
            public static final double SLIP_CURRENT = 400;

            /* Ramps */
            public static final double OPEN_LOOP_RAMP = 0.25;
            public static final double CLOSED_LOOP_RAMP = 0.0;

            /* PID Constants */
            public static final double KP = 0.12; // TODO ROBOT SPECIFIC
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KF = 0.0;
            public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(KP, KI, KD, KF);

            /* Feedforward Constants */
            public static final double KS = 0.32; // TODO ROBOT SPECIFIC
            public static final double KV = 1.51;
            public static final double KA = 0.27;
        }


        public static final class SteerConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = MODULE_TYPE.steerGearRatio;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = MODULE_TYPE.steerMotorInvert;

            /* Neutral Modes */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake; // TODO CHANGE TO BRAKE AFTER MEASURING OFFSETS

            /* Current Limits */
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final int SUPPLY_CURRENT_THRESHOLD = 40;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;        

            /* PID Constants */
            public static final double KP = MODULE_TYPE.steerKP; 
            public static final double KI = MODULE_TYPE.steerKI;
            public static final double KD = MODULE_TYPE.steerKD;
            public static final double KF = MODULE_TYPE.steerKF;
            public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(KP, KI, KD, KF);
        }


        public static class ModuleConstants{

            public record SwerveModuleConstants(int driveMotorID, int steerMotorID, int encoderID, Rotation2d angleOffset){}

            public enum ModuleLocation{
                FRONT_LEFT(0),
                FRONT_RIGHT(1),
                BACK_LEFT(2),
                BACK_RIGHT(3);

                private int number;

                private ModuleLocation(int number){
                    this.number = number;
                }

                public int getNumber() {
                    return number;
                }
            }

            /* Front Left */
            public static final SwerveModuleConstants MODULE_0 = new SwerveModuleConstants(
                23, 
                24, 
                8, 
                Rotation2d.fromRotations(-0.51025390625)); // TODO ROBOT SPECIFIC

            /* Front Right */
            public static final SwerveModuleConstants MODULE_1 = new SwerveModuleConstants(
                13, 
                14, 
                3, 
                Rotation2d.fromRotations(-0.353759765625)); // TODO ROBOT SPECIFIC

            /* Back Left */
            public static final SwerveModuleConstants MODULE_2 = new SwerveModuleConstants(
                19, 
                20, 
                6, 
                Rotation2d.fromRotations(-0.2861328125)); // TODO ROBOT SPECIFIC

            /* Back Right */
            public static final SwerveModuleConstants MODULE_3 = new SwerveModuleConstants(
                17, 
                18, 
                5, 
                Rotation2d.fromRotations(-0.3271484375)); // TODO ROBOT SPECIFIC
        }
    }
}
