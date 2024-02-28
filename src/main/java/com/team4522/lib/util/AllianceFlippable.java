package com.team4522.lib.util;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.PathSequence.Side;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2024.RobotContainer;
import frc2024.Constants.FieldConstants;
import frc2024.subsystems.Vision.IntakePipeline;

public final class AllianceFlippable {

    public static boolean shouldFlip(){
        return RobotContainer.getAlliance() == Alliance.Blue;
    }

    public static Rotation2d getForwardRotation(){
        return shouldFlip() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    public static int getDirectionCoefficient(){
        return (int) Number(1, -1);
    }

    public static Pose2d getTargetSpeaker(){
        return Pose2d(new Pose2d(FieldConstants.BLUE_SPEAKER, getForwardRotation()), new Pose2d(FieldConstants.RED_SPEAKER, getForwardRotation()));
    }

    public static IntakePipeline getIntakePipeline(Side side){
        IntakePipeline pipeline;
        switch (side) {
            case AMP:
            pipeline = (IntakePipeline) Object(IntakePipeline.DETECTOR_LEFTMOST, IntakePipeline.DETECTOR_RIGHTMOST);
            break;
            case SOURCE:
            pipeline = (IntakePipeline) Object(IntakePipeline.DETECTOR_RIGHTMOST, IntakePipeline.DETECTOR_LEFTMOST);
            break;
            default:
            pipeline = IntakePipeline.DETECTOR_LARGEST;
            break;
        }
        return pipeline;
    }

    public static Object Object(Object blueValue, Object redValue){
        return shouldFlip() ? blueValue : redValue;
    }

    public static double Number(double blueValue, double redValue){
        return (double) Object(blueValue, redValue);
    }

    public static Rotation2d Rotation2d(Rotation2d blueValue, Rotation2d redValue){
        return (Rotation2d) Object(blueValue, redValue);
    }

    public static Translation2d Translation2d(Translation2d blueValue, Translation2d redValue){
        return (Translation2d) Object(blueValue, redValue);
    }

    public static Pose2d Pose2d(Pose2d blueValue, Pose2d redValue){
        return (Pose2d) Object(blueValue, redValue);
    }

    public static Rotation2d MirroredRotation2d(Rotation2d blueValue){
        return blueValue.rotateBy(Rotation2d.fromDegrees(180));
    }

    public static Rotation3d MirroredRotation3d(Rotation3d blueValue){
        return blueValue.rotateBy(new Rotation3d(0, 0, Math.PI));
    }

    public static Translation2d MirroredTranslation2d(Translation2d blueValue){
        return new Translation2d(FieldConstants.FIELD_DIMENSIONS.getX() - blueValue.getX(), blueValue.getY());
    }

    public static Pose2d MirroredPose2d(Pose2d blueValue){
        return new Pose2d(MirroredTranslation2d(blueValue.getTranslation()), blueValue.getRotation().unaryMinus());
    }

    public static Translation3d MirroredTranslation3d(Translation3d blueValue){
        Translation2d temp = MirroredTranslation2d(blueValue.toTranslation2d());
        return new Translation3d(temp.getX(), temp.getY(), blueValue.getZ());
    }

    public static Pose3d MirroredPose3d(Pose3d blueValue){
        return new Pose3d(MirroredTranslation3d(blueValue.getTranslation()), MirroredRotation3d(blueValue.getRotation()));
    } 
}
