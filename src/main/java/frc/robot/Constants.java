package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.AllianceFlippable;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.ScreamUtil;

/**
 * A class for constants used in various places in the project.
 */
public final class Constants{

    public record MotionMagicConstants(double cruiseVelocity, double acceleration, int jerk){}

    /* Robot loop time */
    public static final double LOOP_TIME_SEC = 0.02;
    public static final double LOOP_TIME_HZ = 1 / LOOP_TIME_SEC;

    public static final class Ports {
        /** Possible CAN bus strings are:
         *   • "rio" for the native roboRIO CAN bus 
         *   • CANivore name or serial number 
         *   • "*" for any CANivore seen by the program
         */
        public static final String CANIVORE_NAME = "canivore"; // TODO ROBOT SPECIFIC
        public static final String RIO_CANBUS_NAME = "rio";

        public static final String LIMELIGHT_FRONT = "limelight-front";

        /* Pigeon2 */
        public static final int PIGEON_ID = 0; // TODO ROBOT SPECIFIC
        
        /* Shooter */
        public static final int RIGHT_SHOOTER_MOTOR_ID = 0; //TODO
        public static final int LEFT_SHOOTER_MOTOR_ID = 0; //TODO

        /* Pivot */
        public static final int PIVOT_MOTOR_ID = 0; //TODO
        public static final int PIVOT_ENCODER_ID = 0; //TODO

        /* Elevator */
        public static final int LEFT_ELEVATOR_MOTOR_ID = 0; //TODO
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 0; //TODO

        /* Conveyor */
        public static final int CONVEYOR_MOTOR_ID = 0; //TODO

        /* Intake */
        public static final int LEFT_INTAKE_MOTOR_ID = 8;
        public static final int RIGHT_INTAKE_MOTOR_ID = 9;
    }

    
    public static final class ShuffleboardConstants {

        /* For updating values like PID from Shuffleboard */
        public static final boolean UPDATE_SWERVE = false;
    }


    public static final class SwerveConstants {

        /* Drivebase Constants */
        // TODO ROBOT SPECIFIC
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75); // Distance from left wheels to right wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(22.65); // Distance from front wheels to back wheels

        /* Gyro Constants */
        public static final boolean GYRO_INVERT = false;

        /* Swerve Kinematics */
        public static final double MAX_SPEED = 5.0; // m/s theoretical = 5.7
        public static final double MAX_ACCELERATION = 3.0; // m/s^2 theoretical
        public static final double MAX_ANGULAR_VELOCITY = 8.0; // rad/s
        public static final double MAX_ANGULAR_ACCELERATION = 7.679;

        /* Swerve Kinematics */
        // No need to ever change this unless there are more than four modules.
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /* Selected Module Constants */
        public static final COTSFalconSwerveConstants MODULE_TYPE = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.L3); 

        /* Swerve Heading Correction */
        public static final ScreamPIDConstants HEADING_CONSTANTS = new ScreamPIDConstants(0.1, 0.0, 0.001);
        public static final double CORRECTION_TIME_THRESHOLD = 0.2;

        /* Swerve Controllers */
        public static final ScreamPIDConstants VISION_TRANSLATION_X_CONSTANTS = new ScreamPIDConstants(1, 0, 0.0);
        public static final ScreamPIDConstants VISION_TRANSLATION_Y_CONSTANTS = new ScreamPIDConstants(4.5, 0, 0.0);
        public static final ScreamPIDConstants SNAP_CONSTANTS = new ScreamPIDConstants(0.2, 0, 0);

        /* PathPlanner Constants */
        public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS = new ScreamPIDConstants(10.0, 0.0, 0.0);
        public static final ScreamPIDConstants PATH_ROTATION_CONSTANTS = new ScreamPIDConstants(10.0, 0.0, 0.0);

        public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                PATH_TRANSLATION_CONSTANTS.toPathPlannerPIDConstants(), 
                PATH_ROTATION_CONSTANTS.toPathPlannerPIDConstants(), 
                MAX_SPEED, 
                new Translation2d(SwerveConstants.WHEEL_BASE/2, SwerveConstants.TRACK_WIDTH/2).getNorm(), 
                new ReplanningConfig(),
                LOOP_TIME_SEC
        );

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            MAX_SPEED, 
            MAX_ACCELERATION, 
            MAX_SPEED, 
            MAX_ANGULAR_ACCELERATION
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


        public static final class ModuleConstants{

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
                1, 
                0, 
                0, 
                Rotation2d.fromRotations(-0.19970703125)); // TODO ROBOT SPECIFIC

            /* Front Right */
            public static final SwerveModuleConstants MODULE_1 = new SwerveModuleConstants(
                3, 
                2, 
                1, 
                Rotation2d.fromRotations(0.48583984375)); // TODO ROBOT SPECIFIC

            /* Back Left */
            public static final SwerveModuleConstants MODULE_2 = new SwerveModuleConstants(
                5, 
                4, 
                2, 
                Rotation2d.fromRotations(-0.0849609375)); // TODO ROBOT SPECIFIC

            /* Back Right */
            public static final SwerveModuleConstants MODULE_3 = new SwerveModuleConstants(
                7, 
                6, 
                3, 
                Rotation2d.fromRotations(0.17333984375)); // TODO ROBOT SPECIFIC
        }
    }

    public static final class ShooterConstants { //TODO all values
        
        /* Gear Ratio */
        public static final double GEAR_RATIO = 1.0;

        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 25;
        public static final int SUPPLY_CURRENT_THRESHOLD = 40;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;

        public static final double TARGET_THRESHOLD = 0.50;

        public static final double CRUISE_VELOCITY = 40;
        public static final double ACCELERATION = 10;

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(15, 0.0, 0.0);

        public static final double SHOOTER_SHOOT_SPEED = 1.0;
        public static final double SHOOTER_EJECT_SPEED = 0.5;
    }
    
    public static final class PivotConstants { //TODO all values
        
        /* Gear Ratio */
        public static final double GEAR_RATIO = 1.0;

        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 25;
        public static final int SUPPLY_CURRENT_THRESHOLD = 40;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final boolean SOFTWARE_LIMIT_ENABLE = true;
        public static final double FORWARD_SOFT_LIMIT = 0.0;
        public static final double REVERSE_SOFT_LIMIT = 0.0;

        public static final double TARGET_THRESHOLD = 0.50;

        public static final double CRUISE_VELOCITY = 40;
        public static final double ACCELERATION = 10;

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(15, 0.0, 0.0);

        public static final Rotation2d PIVOT_HOME_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d PIVOT_SUBWOOFER_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d PIVOT_AMP_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d PIVOT_TRAP_ANGLE = Rotation2d.fromDegrees(0.0);

        public static final InterpolatingDoubleTreeMap pivotTreeMap = new InterpolatingDoubleTreeMap();
        static{
            //pivotTreeMap.put(distance, angle (degrees))
            pivotTreeMap.put(null, null);
        }
    }

    public static final class ElevatorConstants { //TODO all values

        /* Gear Ratio */
        public static final double GEAR_RATIO = 1.0;

        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 25;
        public static final int SUPPLY_CURRENT_THRESHOLD = 40;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final boolean SOFTWARE_LIMIT_ENABLE = true;
        public static final double FORWARD_SOFT_LIMIT = 0.0;
        public static final double REVERSE_SOFT_LIMIT = 0.0;

        public static final double TARGET_THRESHOLD = 0.50;

        public static final double CRUISE_VELOCITY = 40;
        public static final double ACCELERATION = 10;

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(15, 0.0, 0.0);

        public static final double ELEVATOR_HOME_POSITION = 0.0;
        public static final double ELEVATOR_SUBWOOFER_POSITION = 0.0;
        public static final double ELEVATOR_AMP_POSITION = 0.0;
        public static final double ELEVATOR_TRAP_POSITION = 0.0;

        public static final InterpolatingDoubleTreeMap elevatorTreeMap = new InterpolatingDoubleTreeMap();
        static{
            //elevatorTreeMap.put(distance, height)
            elevatorTreeMap.put(null, null);
        }
    }

    public static final class IntakeConstants { //TODO all values

        /* Gear Ratio */
        public static final double GEAR_RATIO = 1.0;
        
        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 25;
        public static final int SUPPLY_CURRENT_THRESHOLD = 40;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;

        public static final double INTAKE_SPEED = -0.75;
        public static final double EJECT_SPEED = 0.5;
    }

    public static final class ConveyorConstants {//TODO all values
        
        /* Gear Ratio */
        public static final double GEAR_RATIO = 1.0;
        
        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 25;
        public static final int SUPPLY_CURRENT_THRESHOLD = 40;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final double SPEAKER_SPEED = 1.00;
        public static final double AMP_TRAP_SPEED = -1.00;
        public static final double TRANSFER_SPEED = 0.75;
    }

    public static final class VisionConstants {
        public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.5);

        public static final int DETECTOR_PIPELINE = 0;

        public static final double DETECTOR_TARGET_TY = -10.0;
        public static final double DETECTOR_TARGET_TX = 0.0;
        public static final double VALID_TARGET_THRESHOLD = 0;
    }

    public static final class FieldConstants{

        public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.54175, 8.21055);
        // Numbered from amp station wall
        public static final Translation2d CENTER_PIECE_1 = new Translation2d(8.256, 7.456);
        public static final Translation2d CENTER_PIECE_2 = new Translation2d(8.256, 5.788);
        public static final Translation2d CENTER_PIECE_3 = new Translation2d(8.256, 4.112);
        public static final Translation2d CENTER_PIECE_4 = new Translation2d(8.256, 2.436);
        public static final Translation2d CENTER_PIECE_5 = new Translation2d(8.256, 0.759);

        public static final Translation2d BLUE_PIECE_1 = new Translation2d(2.882, 7.001);
        public static final Translation2d BLUE_PIECE_2 = new Translation2d(2.882, 5.553);
        public static final Translation2d BLUE_PIECE_3 = new Translation2d(2.882, 4.106);

        public static final Translation2d RED_PIECE_1 = new Translation2d(13.631, 7.001);
        public static final Translation2d RED_PIECE_2 = new Translation2d(13.631, 5.553);
        public static final Translation2d RED_PIECE_3 = new Translation2d(13.631, 4.106);

        public static final Translation2d ROBOT_OFFSET = new Translation2d(Units.inchesToMeters(38.0), 0);

        public static final Translation2d[] CENTER_PIECE_TRANSLATIONS = new Translation2d[]{
            CENTER_PIECE_1,
            CENTER_PIECE_2,
            CENTER_PIECE_3,
            CENTER_PIECE_4,
            CENTER_PIECE_5,
        };

        public static final Translation2d BLUE_PODIUM = new Translation2d(2.737, 4.131913);
        public static final Translation2d RED_PODIUM = new Translation2d(13.816724, 4.131913);

        public static final Translation2d BLUE_SPEAKER_OPENING = new Translation2d(0.0, 5.54);
        public static final Translation2d RED_SPEAKER_OPENING = AllianceFlippable.MirroredTranslation2d(BLUE_SPEAKER_OPENING);

        public static final double SPEAKER_TAGS_HEIGHT = 1.440488;
    }
}
