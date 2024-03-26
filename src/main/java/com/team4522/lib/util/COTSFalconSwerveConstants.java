package com.team4522.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSFalconSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double steerGearRatio;
    public final double driveGearRatio;
    public final double steerKP;
    public final double steerKI;
    public final double steerKD;
    public final double steerKF;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue steerMotorInvert;
    public final SensorDirectionValue CANcoderInvert;

    public COTSFalconSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP,
            double angleKI, double angleKD, double angleKF, InvertedValue driveMotorInvert, InvertedValue angleMotorInvert,
            SensorDirectionValue CANcoderInvert) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.steerGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.steerKP = angleKP;
        this.steerKI = angleKI;
        this.steerKD = angleKD;
        this.steerKF = angleKF;
        this.driveMotorInvert = driveMotorInvert;
        this.steerMotorInvert = angleMotorInvert;
        this.CANcoderInvert = CANcoderInvert;
    }

    /** Swerve Drive Specialties - MK3 Module */
    public static COTSFalconSwerveConstants SDSMK3(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 9.6;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive; //CounterClockwise_Positive = false; Clockwise_Positive = true
        InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        SensorDirectionValue CANcoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD,
                angleKF, driveMotorInvert, angleMotorInvert, CANcoderInvert);
    }

    /** Swerve Drive Specialties - MK4 Module */
    public static COTSFalconSwerveConstants SDSMK4(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 9.6;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive; //CounterClockwise_Positive = false; Clockwise_Positive = true
        InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        SensorDirectionValue CANcoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD,
                angleKF, driveMotorInvert, angleMotorInvert, CANcoderInvert);
    }

    /** Swerve Drive Specialties - MK4i Module */
    public static COTSFalconSwerveConstants SDSMK4i(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0 - 0.5);

        /** 21.42 : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive; //CounterClockwise_Positive = false; Clockwise_Positive = true
        InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        SensorDirectionValue CANcoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD,
                angleKF, driveMotorInvert, angleMotorInvert, CANcoderInvert);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios {
        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double MK3_Standard = (8.16 / 1.0);
        /** SDS MK3 - 6.86 : 1 */
        public static final double MK3_Fast = (6.86 / 1.0);

        /* MK4 Modules */
        /** L1 - 8.14 : 1 */
        public static final double L1 = (8.14 / 1.0);
        /** L2 - 6.75 : 1 */
        public static final double L2 = (6.75 / 1.0);
        /** L3 - 6.12 : 1 */
        public static final double L3 = (6.12 / 1.0);
        /** L4 - 5.14 : 1 */
        public static final double L4 = (5.14 / 1.0);
    }
}
