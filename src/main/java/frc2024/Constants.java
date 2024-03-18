package frc2024;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.AllianceFlippable;
import com.team4522.lib.util.COTSFalconSwerveConstants;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootStateInterpolatingTreeMap;

import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc2024.subsystems.Pivot;

/**
 * A class for constants used in various places in the project.
 */
public final class Constants{

    public record MotionMagicConstants(double cruiseVelocity, double acceleration, int jerk){}
    public record FeedforwardConstants(double kV, double kS, double kG, double kA, GravityTypeValue gravityType){
        public FeedforwardConstants(double kV, double kS, double kG, double kA){
            this(kV, kS, kG, kA, GravityTypeValue.Elevator_Static);
        }
        public FeedforwardConstants(){
            this(0, 0, 0, 0, GravityTypeValue.Elevator_Static);
        }
    }
    public enum Mode{
        REAL, REPLAY, SIM, DEV;
    }

    public static final Mode MODE = Mode.REAL;

    /* Robot loop time */
    public static final double LOOP_TIME_SEC = 0.02;
    public static final double LOOP_TIME_HZ = 1 / LOOP_TIME_SEC;
    public static final double DEVICE_LOOP_TIME_HZ = 100.0;

    public static final class Ports {
        /** Possible CAN bus strings are:
         *   • "rio" for the native roboRIO CAN bus 
         *   • CANivore name or serial number 
         *   • "*" for any CANivore seen by the program
         */
        public static final String CANIVORE_NAME = "canivore"; // TODO ROBOT SPECIFIC
        public static final String RIO_CANBUS_NAME = "rio";

        public static final int LED_ID = 9;

        /* Misc */
        public static final int PIGEON_ID = 0; // TODO ROBOT SPECIFIC
        public static final int CANDLE_ID = 0;
        
        /* Stabilizers */
        // public static final int LEFT_CLIMBER_MOTOR_ID = 13; //TODO
        public static final int BAR_MOTOR_ID = 13;
        //public static final int RIGHT_CLIMBER_MOTOR_ID = 14; //TODO

        /* Shooter */
        public static final int BOTTOM_SHOOTER_MOTOR_ID = 12; //TODO
        public static final int TOP_SHOOTER_MOTOR_ID = 11; //TODO

        /* Pivot */
        public static final int PIVOT_MOTOR_ID = 17; //TODO
        public static final int PIVOT_ENCODER_ID = 4; //TODO

        /* Elevator */
        public static final int LEFT_ELEVATOR_MOTOR_ID = 16; //TODO
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 15; //TODO
        public static final int ELEVATOR_ENCODER_ID = 5;

        /* Conveyor */
        public static final int CONVEYOR_MOTOR_ID = 10; //TODO
        public static final int CONVEYOR_BEAM_ID = 2; // TODO

        /* Intake */
        //public static final int LEFT_INTAKE_MOTOR_ID = 9;
        public static final int RIGHT_INTAKE_MOTOR_ID = 9;
        public static final int INTAKE_BEAM_ID = 0;
    }

    
    public static final class ShuffleboardConstants {

        /* For updating values like PID from Shuffleboard */
        public static final boolean UPDATE_SWERVE = false;
        public static final boolean UPDATE_INTAKE = false;
        public static final boolean UPDATE_SHOOTER = false;
        public static final boolean UPDATE_ELEVATOR = false;
        public static final boolean UPDATE_CONVEYOR = false;
        public static final boolean UPDATE_CLIMBER = false;
        public static final boolean UPDATE_PIVOT = false;
    }

    public static final class SwerveConstants {

        /* Drivebase Constants */
        // TODO ROBOT SPECIFIC
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75); // Distance from left wheels to right wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(20.75); // Distance from front wheels to back wheels

        /* Gyro Constants */
        public static final boolean GYRO_INVERT = false;

        /* Swerve Kinematics */
        public static final double SHOOT_WHILE_MOVING_SCALAR = 0.5;
        public static final double MAX_SPEED = 4.9; // m/s theoretical = 5.7
        public static final double MAX_ACCELERATION = 4.9; // m/s^2 theoretical
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

        public static final SwerveDriveKinematics POSE_ESTIMATOR_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE + Units.inchesToMeters(5 - 2.75) / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE + Units.inchesToMeters(5 - 2.75) / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /* Selected Module Constants */
        public static final COTSFalconSwerveConstants MODULE_TYPE = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.L3); 

        /* Swerve Heading Correction */
        public static final ScreamPIDConstants HEADING_CONSTANTS = new ScreamPIDConstants(0.1, 0.0, 0.001);
        public static final double CORRECTION_TIME_THRESHOLD = 0.2;

        /* Swerve Controllers */
        public static final ScreamPIDConstants VISION_ROTATION_CONSTANTS = new ScreamPIDConstants(0.04, 0.0, 0.0);
        public static final ScreamPIDConstants VISION_TRANSLATION_X_CONSTANTS = new ScreamPIDConstants(1.0, 0.0, 0.0);
        public static final ScreamPIDConstants VISION_TRANSLATION_Y_CONSTANTS = new ScreamPIDConstants(4.5, 0.0, 0.0);
        public static final ScreamPIDConstants SNAP_CONSTANTS = new ScreamPIDConstants(0.095, 0.0, 0.0); //0.1
        public static final ScreamPIDConstants DRIVE_TO_TARGET_CONSTANTS = new ScreamPIDConstants(1.5, 0.0, 0.0);

        /* PathPlanner Constants */
        public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS = new ScreamPIDConstants(25.0, 0.0, 0.0);
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
            public static final int SUPPLY_CURRENT_LIMIT = 40;
            public static final int SUPPLY_CURRENT_THRESHOLD = 70;
            public static final double SUPPLY_TIME_THRESHOLD = 1.0;
            public static final boolean SUPPLY_LIMIT_ENABLE = true;
            public static final double SLIP_CURRENT = 400;
            public static final int STATOR_CURRENT_LIMIT = 80;
            public static final boolean STATOR_LIMIT_ENABLE = true;

            /* Ramps */
            public static final double OPEN_LOOP_RAMP = 0.0;
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
            public static final double KG = 0.0;
            public static final FeedforwardConstants FEEDFORWARD_CONSTANTS = new FeedforwardConstants(KV, KS, KG, KA);
        }

        public static final class SteerConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = MODULE_TYPE.steerGearRatio;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = MODULE_TYPE.steerMotorInvert;

            /* Neutral Modes */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

            /* Current Limits */
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final int SUPPLY_CURRENT_THRESHOLD = 40;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;   
            public static final int STATOR_CURRENT_LIMIT = 80;
            public static final boolean STATOR_LIMIT_ENABLE = true;     

            /* PID Constants */
            public static final double KP = MODULE_TYPE.steerKP; 
            public static final double KI = MODULE_TYPE.steerKI;
            public static final double KD = MODULE_TYPE.steerKD;
            public static final double KF = MODULE_TYPE.steerKF;
            public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(KP, KI, KD, KF);

            public static final double CRUISE_VELOCITY = 40.0;
            public static final double ACCELERATION = 30.0;
            public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        }


        public static final class ModuleConstants{

            public record SwerveModuleConstants(int driveMotorID, int steerMotorID, int encoderID, Rotation2d angleOffset){}

            public enum ModuleLocation{
                FRONT_LEFT(0),
                FRONT_RIGHT(1),
                BACK_LEFT(2),
                BACK_RIGHT(3);

                public int number;

                private ModuleLocation(int number){
                    this.number = number;
                }
            }

            /* Front Left */
            public static final SwerveModuleConstants MODULE_0 = new SwerveModuleConstants(
                1, 
                0, 
                0, 
                Rotation2d.fromRotations(-0.211669921875) /* -0.319580078125 */
            );

            /* Front Right */
            public static final SwerveModuleConstants MODULE_1 = new SwerveModuleConstants(
                3, 
                2, 
                1, 
                Rotation2d.fromRotations(-0.578125) /* -0.0546875 */
            );

            /* Back Left */
            public static final SwerveModuleConstants MODULE_2 = new SwerveModuleConstants(
                5, 
                4, 
                2, 
                Rotation2d.fromRotations(-0.073974609375) /* 0.23974609375 */
            );

            /* Back Right */
            public static final SwerveModuleConstants MODULE_3 = new SwerveModuleConstants(
                7, 
                6, 
                3, 
                Rotation2d.fromRotations(-0.50439453125) /* -0.26318359375+0.5 */
            );

            public static final SwerveModuleConstants MODULE_4 = new SwerveModuleConstants(
                3, 
                2, 
                1, 
                Rotation2d.fromRotations(-0.572998046875)
            );

            public static final SwerveModuleConstants MODULE_5 = new SwerveModuleConstants(
                2, 
                3, 
                1, 
                Rotation2d.fromRotations(0.0)
            );
        }
    }

    public static final class ClimberConstants { //TODO all values
        
        /* Gear Ratio */
        public static final double GEAR_RATIO = 16.0; //36.0;

        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 10;
        public static final int SUPPLY_CURRENT_THRESHOLD = 10;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final boolean SOFTWARE_LIMIT_ENABLE = false;
        public static final double FORWARD_SOFT_LIMIT = 0.0;
        public static final double REVERSE_SOFT_LIMIT = 0.0;

        public static final double TARGET_THRESHOLD = 0.50; //m

        public static final double CRUISE_VELOCITY = 40;
        public static final double ACCELERATION = 10;

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(15, 0.0, 0.0);

        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
        public static final double KG = 0.0;
        public static final FeedforwardConstants FEEDFORWARD_CONSTANTS = new FeedforwardConstants(KV, KS, KG, KA);

        public static final double CLIMBER_UP_OUTPUT = 0.1;

        /* public static final class BarConstants{
            public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(0.01, 0, 0);

            public static final int SUPPLY_CURRENT_LIMIT = 10;
            public static final int SUPPLY_CURRENT_THRESHOLD = 15;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;
        } */
    }

    public static final class ShooterConstants { //TODO all values
        
        /* Gear Ratio */
        public static final double GEAR_RATIO = 1.0;

        /* Circumference */
        public static final double WHEEL_CIRCUMFERENCE = 4.0 * Math.PI;

        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 40;
        public static final int SUPPLY_CURRENT_THRESHOLD = 50;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;

        public static final double CRUISE_VELOCITY = 10;
        public static final double ACCELERATION = 5;

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(0.38, 0.01, 0.0);

        public static final double KS = 0.0;
        public static final double KV = 0.13;
        public static final double KA = 0.0;
        public static final double KG = 0.0;
        public static final FeedforwardConstants FEEDFORWARD_CONSTANTS = new FeedforwardConstants(KV, KS, KG, KA);

        public static final double TARGET_THRESHOLD = 150.0; // rpm

        public static final double AUTO_SHOOT_VELOCITY_THRESHOLD = 4800; // rpm
        public static final double AUTO_SHOOT_DISTANCE_THRESHOLD = 6.5; // meters

        public static final double SHOOTER_MAX_VELOCITY = 6100;
        public static final double SHOOTER_TARGET_VELOCITY = 5000;
        public static final double SUBWOOFER_VELOCITY = 2500;
        public static final double AMP5_CLOSESHOTS = 3250.0;
        public static final double SUBWOOFER_DEFENDED_VELOCITY = 3000;
        public static final double CHAIN_VELOCITY = 4000.0;
        public static final double PODIUM_VELOCITY = 3000.0;
        public static final double IDLE_VELOCITY = 1000.0;

        public static final double TRAJECTORY_VELOCITY_EXTRA = 2000.0;

        public static final double SHOOT_OUTPUT = 0.8;
        public static final double EJECT_OUTPUT = 0.25;//0.5;
    }
    
    public static final class PivotConstants { //TODO all values
        
        /* Gear Ratio */
        public static final double ROTOR_TO_SENSOR_RATIO = 125.0;
        public static final double SENSOR_TO_MECH_RATIO = 72.0 / 22.0;
        public static final double TOTAL_GEAR_RATIO = ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECH_RATIO;

        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 35;
        public static final int SUPPLY_CURRENT_THRESHOLD = 60;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final boolean SOFTWARE_LIMIT_ENABLE = false;
        public static final boolean SOFTWARE_LIMIT_ENABLE_ENDGAME = true;
        public static final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromDegrees(40.0);
        public static final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromDegrees(-9999);
        public static final Rotation2d FORWARD_SOFT_LIMIT_ENDGAME = Rotation2d.fromDegrees(54.0);
        public static final Rotation2d REVERSE_SOFT_LIMIT_ENDGAME = Rotation2d.fromDegrees(25.0);

        public static final boolean HARDWARE_AUTO_POSITION_FORWARD_ENABLE = false;
        public static final boolean HARDWARE_AUTO_POSITION_REVERSE_ENABLE = false;
        public static final double HARDWARE_LIMIT_POSITION_FORWARD = 0.0;
        public static final double HARDWARE_LIMIT_POSITION_REVERSE = 0.0;

        public static final double TARGET_THRESHOLD = 0.2; //Degrees

        public static final double CRUISE_VELOCITY = 200;
        public static final double ACCELERATION = 200;

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(500.0, 0.0, 0.0); //500
        
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
        public static final double KG = 0.0;
        public static final FeedforwardConstants FEEDFORWARD_CONSTANTS = new FeedforwardConstants(KV, KS, KG, KA, GravityTypeValue.Arm_Cosine);

        public static final Rotation2d HOME_ANGLE = Rotation2d.fromDegrees(14.0);
        public static final Rotation2d HOME_ANGLE_ENDGAME = Rotation2d.fromDegrees(-13.623);
        public static final Rotation2d SUBWOOFER_ANGLE = Rotation2d.fromDegrees(-5.3613-1.5);
        public static final Rotation2d SUBWOOFER_ANGLE_DEFENDED = Rotation2d.fromDegrees(16.9629); //+4.25
        public static final Rotation2d AMP_ANGLE = Rotation2d.fromDegrees(29.0039);
        public static final Rotation2d CHAIN_ANGLE = Rotation2d.fromDegrees(22.93945); //+4.25
        public static final Rotation2d PODIUM_ANGLE = Rotation2d.fromDegrees(13.7988);
        public static final Rotation2d PODIUM_DEFENDED_ANGLE = Rotation2d.fromDegrees(24.1699);
        public static final Rotation2d TRAP_CHAIN_ANGLE = Rotation2d.fromDegrees(48.516 /* 56.0742 */);
        public static final Rotation2d AMP5_CLOSESHOTS = Rotation2d.fromDegrees(16.25);

        public static final double AUTO_ZERO_OUTPUT = 0.0;

        public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromRotations(0.31201171875);
        public static final Rotation2d RELATIVE_ENCODER_TO_HORIZONTAL = Rotation2d.fromDegrees(44.824); // TODO RE-MEASURE

        public static final double AXLE_HEIGHT_HOME = Units.inchesToMeters(16.640069);
        public static final double AXLE_HEIGHT_TOP = Units.inchesToMeters(38.059638);
        public static final double AXLE_DISTANCE_FROM_ELEVATOR_TOP = Units.inchesToMeters(9.975600);
        public static final double AXLE_DISTANCE_FROM_LENS_HOME = Units.inchesToMeters(9.189772);
        public static final double AXLE_DISTANCE_FROM_LENS_TOP = Units.inchesToMeters(12.966620);
        public static final double SHOOTER_DISTANCE_FROM_AXLE = Units.inchesToMeters(3.100600);
    }

    public static final class ElevatorConstants { //TODO all values

        /* Gear Ratio */
        public static final double ROTOR_TO_SENSOR_RATIO = 14.0167; //6.0;
        public static final double SENSOR_TO_MECH_RATIO = ROTOR_TO_SENSOR_RATIO/6.0;//1.0;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 35;
        public static final int SUPPLY_CURRENT_THRESHOLD = 50;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final boolean SOFTWARE_LIMIT_ENABLE = true;
        public static final double FORWARD_SOFT_LIMIT = 3.1;
        public static final double REVERSE_SOFT_LIMIT = 0.0;

        public static final double CRUISE_VELOCITY = 50*(ROTOR_TO_SENSOR_RATIO/3.0);
        public static final double ACCELERATION = 30*(ROTOR_TO_SENSOR_RATIO/9.0);

        public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS = new MotionMagicConstants(CRUISE_VELOCITY, ACCELERATION, 0);
        public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(50.0*(ROTOR_TO_SENSOR_RATIO/9.0), 0.0, 0.0);//50
        
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
        public static final double KG = 0.3; //0.415;
        public static final FeedforwardConstants FEEDFORWARD_CONSTANTS = new FeedforwardConstants(KV, KS, KG, KA, GravityTypeValue.Elevator_Static);

        public static final double TARGET_THRESHOLD = 0.75; // inches

        public static final double MAX_HEIGHT = 21.75; // inches
        public static final double MIN_HEIGHT = 0.0; // inches
        public static final double ENCODER_MAX = 3.1; // relative
        public static final double ENCODER_MIN = 0.0; // relative
        public static final double HOME_HEIGHT_FROM_FLOOR = Units.inchesToMeters(26.553805);

        public static final double HOME_HEIGHT = 0.0;
        public static final double HOME_HEIGHT_ENDGAME = 3.408;
        public static final double SUBWOOFER_HEIGHT = 3.12684; //3.5;
        public static final double AMP_HEIGHT = 19.70989;
        public static final double TRAP_CHAIN_HEIGHT = MAX_HEIGHT;

        public static final double REHOME_VOLTAGE = -5.0;
        public static final double REHOME_CURRENT_THRESHOLD = 0.0;

        public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(-0.400634765625);
    }

    public static final class IntakeConstants { //TODO all values

        /* Gear Ratio */
        public static final double GEAR_RATIO = 3.0;
        
        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 25;
        public static final int SUPPLY_CURRENT_THRESHOLD = 40;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;

        public static final double INTAKE_OUTPUT = 0.8;
        public static final double EJECT_OUTPUT = -0.5;
    }

    public static final class ConveyorConstants {//TODO all values
        
        /* Gear Ratio */
        public static final double GEAR_RATIO = 4.0;
        
        /* Motor Invert */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;;
        
        /* Neutral Modes */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        
        /* Current Limits */
        public static final int SUPPLY_CURRENT_LIMIT = 35;
        public static final int SUPPLY_CURRENT_THRESHOLD = 60;
        public static final double SUPPLY_TIME_THRESHOLD = 0.1;
        public static final boolean CURRENT_LIMIT_ENABLE = true;
        
        public static final double SHOOT_SPEED = 1.00;
        public static final double AMP_OUTPUT = -1.0;
        public static final double TRANSFER_OUTPUT = 0.75;
        public static final double TRAP_SPEED = 0.5;
    } 

    public static enum SuperstructureState{
        NONE(ElevatorConstants.HOME_HEIGHT, PivotConstants.HOME_ANGLE),
        HOME(ElevatorConstants.HOME_HEIGHT, PivotConstants.HOME_ANGLE), 
        HOME_ENDGAME(ElevatorConstants.HOME_HEIGHT_ENDGAME, PivotConstants.HOME_ANGLE_ENDGAME), 
        AMP(ElevatorConstants.AMP_HEIGHT, PivotConstants.AMP_ANGLE), 
        SUBWOOFER(ElevatorConstants.SUBWOOFER_HEIGHT, PivotConstants.SUBWOOFER_ANGLE),
        CHAIN(ElevatorConstants.HOME_HEIGHT, PivotConstants.CHAIN_ANGLE),
        PODIUM(ElevatorConstants.HOME_HEIGHT, PivotConstants.PODIUM_ANGLE),
        PODIUM_DEFENDED(ElevatorConstants.MAX_HEIGHT, PivotConstants.PODIUM_DEFENDED_ANGLE),
        TRAP_CHAIN(ElevatorConstants.TRAP_CHAIN_HEIGHT, PivotConstants.TRAP_CHAIN_ANGLE),
        SUBWOOFER_DEFENDED(ElevatorConstants.MAX_HEIGHT, PivotConstants.SUBWOOFER_ANGLE_DEFENDED),
        AUTO_FIRE(ElevatorConstants.HOME_HEIGHT, PivotConstants.HOME_ANGLE),
        AMP5_CLOSESHOTS(ElevatorConstants.HOME_HEIGHT, PivotConstants.AMP5_CLOSESHOTS);

        public double elevatorPosition;
        public Rotation2d pivotAngle;
        public boolean limitPivotMotion;

        private SuperstructureState(double elevatorPosition, Rotation2d pivotAngle){
            this.elevatorPosition = elevatorPosition;
            this.pivotAngle = pivotAngle;
        }
    }

    public static record ShootState(Rotation2d pivotAngle, double elevatorHeightInches, double velocityRPM){}

    public static final class VisionConstants {
        public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.003, 0.003, 0.9);
        public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.9, 0.9, 0.001);

        public static final double DETECTOR_TARGET_Y = 0.0;
        public static final double DETECTOR_TARGET_X = 0.0;
        public static final double AUTO_FIRE_X_THRESHOLD = 0.0;
        public static final double AUTO_INTAKE_Y_THRESHOLD = 0.0;

        public static final Rotation2d SHOOT_STATE_MAP_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        public static final double SHOOT_STATE_MAP_ELEVATOR_OFFSET = 0.0;
        public static final double SHOOT_STATE_MAP_VELOCITY_OFFSET = 0.0;

        public static final ShootStateInterpolatingTreeMap SHOOT_STATE_MAP = new ShootStateInterpolatingTreeMap();
        static{
            SHOOT_STATE_MAP.put(3.9103, new ShootState(PivotConstants.SUBWOOFER_ANGLE, ElevatorConstants.SUBWOOFER_HEIGHT, ShooterConstants.SUBWOOFER_VELOCITY));
            SHOOT_STATE_MAP.put(4.9169, new ShootState(Rotation2d.fromDegrees(6.24), 0.0, 2750.0));
            SHOOT_STATE_MAP.put(5.9041, new ShootState(Rotation2d.fromDegrees(8.9648), 0.0, 2850.0));
            SHOOT_STATE_MAP.put(6.9411, new ShootState(Rotation2d.fromDegrees(13.7109), 0.0, 2950.0));
            SHOOT_STATE_MAP.put(7.9108, new ShootState(Rotation2d.fromDegrees(18.1933), 0.0, 3100.0));
            SHOOT_STATE_MAP.put(8.9512, new ShootState(Rotation2d.fromDegrees(20.5664), 0.0, 3250.0));
            SHOOT_STATE_MAP.put(9.9538, new ShootState(Rotation2d.fromDegrees(23.8183), 0.0, 3500.0));
            SHOOT_STATE_MAP.put(11.0, new ShootState(Rotation2d.fromDegrees(23.6425), 0.0, 3600.0));
            SHOOT_STATE_MAP.put(12.0, new ShootState(Rotation2d.fromDegrees(28.0371-2), 0.0, 3800.0));
            SHOOT_STATE_MAP.put(13.0, new ShootState(Rotation2d.fromDegrees(28.7402), 0.0, 4000.0));
            //^^^
            SHOOT_STATE_MAP.put(14.0, new ShootState(Rotation2d.fromDegrees(33.5742-3.25), 0.0, 4150.0));
            SHOOT_STATE_MAP.put(15.0, new ShootState(Rotation2d.fromDegrees(35.9472-4), 0.0, 4250.0));
            SHOOT_STATE_MAP.put(16.0, new ShootState(Rotation2d.fromDegrees(39.7265-6), 0.0, 4600.0));
            SHOOT_STATE_MAP.put(17.0, new ShootState(Rotation2d.fromDegrees(41.2207-6.5), 0.0, 4900.0));
            SHOOT_STATE_MAP.put(18.0, new ShootState(Rotation2d.fromDegrees(41.3964-6), 0.0, 5000.0));
        }

        public static final ShootStateInterpolatingTreeMap SHOOT_STATE_MAP_DEFENDED = new ShootStateInterpolatingTreeMap();
        static{
            SHOOT_STATE_MAP_DEFENDED.put(4.9169, new ShootState(Rotation2d.fromDegrees(14.47), ElevatorConstants.MAX_HEIGHT, 2250.0));
            SHOOT_STATE_MAP_DEFENDED.put(5.9041, new ShootState(Rotation2d.fromDegrees(20.127), ElevatorConstants.MAX_HEIGHT, 2500.0));
            SHOOT_STATE_MAP_DEFENDED.put(6.9411, new ShootState(Rotation2d.fromDegrees(23.906), ElevatorConstants.MAX_HEIGHT, 2750.0));
            SHOOT_STATE_MAP_DEFENDED.put(7.9108, new ShootState(Rotation2d.fromDegrees(26.8924), ElevatorConstants.MAX_HEIGHT, 3000.0));
            SHOOT_STATE_MAP_DEFENDED.put(8.9512, new ShootState(Rotation2d.fromDegrees(30.2343), ElevatorConstants.MAX_HEIGHT, 3250.0));
            SHOOT_STATE_MAP_DEFENDED.put(9.9538, new ShootState(Rotation2d.fromDegrees(33.0468), ElevatorConstants.MAX_HEIGHT, 3500.0));
            SHOOT_STATE_MAP_DEFENDED.put(11.0, new ShootState(Rotation2d.fromDegrees(31.9921), ElevatorConstants.MAX_HEIGHT, 3600.0));
            SHOOT_STATE_MAP_DEFENDED.put(12.0, new ShootState(Rotation2d.fromDegrees(35.9472), ElevatorConstants.MAX_HEIGHT, 3800.0));
            SHOOT_STATE_MAP_DEFENDED.put(13.0, new ShootState(Rotation2d.fromDegrees(36.6503), ElevatorConstants.MAX_HEIGHT, 3950.0));
            SHOOT_STATE_MAP_DEFENDED.put(14.0, new ShootState(Rotation2d.fromDegrees(39.4628), 20.5127, 4100.0));
            SHOOT_STATE_MAP_DEFENDED.put(15.0, new ShootState(Rotation2d.fromDegrees(40.7812), 20.0, 4200.0));
        }
    }

    public static final class LEDConstants{
        public static final int STRIP_LENGTH = 45;

        public static final double BREATHE_DURATION = 1.0;
        public static final double WAVE_EXPONENT = 0.4;
    }

    public static final class FieldConstants{

        public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.541, 8.211);
        
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

        public static final Translation2d[] CENTER_PIECE_TRANSLATIONS = new Translation2d[]{
            CENTER_PIECE_1,
            CENTER_PIECE_2,
            CENTER_PIECE_3,
            CENTER_PIECE_4,
            CENTER_PIECE_5,
        };

        public static final Translation2d BLUE_PODIUM = new Translation2d(2.737, 4.131913);
        public static final Translation2d RED_PODIUM = AllianceFlippable.MirroredTranslation2d(BLUE_PODIUM);
        
        public static final Translation2d BLUE_STAGE_LEFT = new Translation2d(4.65, 3.73);
        public static final Translation2d BLUE_STAGE_MID = new Translation2d(5.33, 4.19);
        public static final Translation2d BLUE_STAGE_RIGHT = new Translation2d(4.65, 4.51);
        public static final Translation2d RED_STAGE_LEFT = AllianceFlippable.MirroredTranslation2d(BLUE_STAGE_LEFT);
        public static final Translation2d RED_STAGE_MID = AllianceFlippable.MirroredTranslation2d(BLUE_STAGE_MID);
        public static final Translation2d RED_STAGE_RIGHT = AllianceFlippable.MirroredTranslation2d(BLUE_STAGE_RIGHT);

        public static final Translation2d BLUE_SPEAKER = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(7).get().getTranslation().toTranslation2d();
        public static final Translation2d RED_SPEAKER = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(4).get().getTranslation().toTranslation2d();

        public static final double SPEAKER_OPENING_HEIGHT = Units.inchesToMeters(80.567496); // 80.567496

        public static final double SPEAKER_TAG_HEIGHT = 1.468864;
        public static final double STAGE_TAG_HEIGHT = 1.320884;
        public static final double SOURCE_TAG_HEIGHT = 1.355726;
        public static final double AMP_TAG_HEIGHT = SOURCE_TAG_HEIGHT;
    }
}