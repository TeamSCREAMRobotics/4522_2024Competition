package com.team4522.lib.util;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.node.POJONode;
import com.pathplanner.lib.util.GeometryUtil;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.PathSequence.Side;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2024.RobotContainer;
import frc2024.Constants.FieldConstants;
import frc2024.subsystems.Vision.IntakePipeline;

public final class AllianceFlipUtil {

    public static BooleanSupplier shouldFlip(){
        return () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static Rotation2d getForwardRotation(){
        return Rotation2d(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
    }

    public static int getDirectionCoefficient(){
        return (int) Number(1, -1);
    }

    public static Pose2d getTargetSpeaker(){
        return Pose2d(new Pose2d(FieldConstants.BLUE_SPEAKER, getForwardRotation()), new Pose2d(FieldConstants.RED_SPEAKER, getForwardRotation()));
    }

    public static Pose2d MirrorPoseForField2d(Pose2d pose){
        double x = pose.getX();
        double y = FieldConstants.FIELD_DIMENSIONS.getY() - pose.getY();
        Rotation2d z = pose.getRotation().unaryMinus();
        return new Pose2d(x, y, z);
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
        return shouldFlip().getAsBoolean() ? redValue : blueValue;
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

    public static Translation3d Translation3d(Translation3d blueValue, Translation3d redValue){
        return (Translation3d) Object(blueValue, redValue);
    }

    public static Pose2d Pose2d(Pose2d blueValue, Pose2d redValue){
        return (Pose2d) Object(blueValue, redValue);
    }

    public static Rotation2d MirroredRotation2d(Rotation2d blueValue){
        return Rotation2d(blueValue, new Rotation2d(Math.PI).minus(blueValue));
    }

    public static Rotation3d MirroredRotation3d(Rotation3d blueValue){
        return new Rotation3d(blueValue.getX(), blueValue.getY(), MirroredRotation2d(Rotation2d.fromRadians(blueValue.getZ())).getRadians());
    }

    public static Translation2d MirroredTranslation2d(Translation2d blueValue){
        return Translation2d(blueValue, new Translation2d(FieldConstants.FIELD_DIMENSIONS.getX() - blueValue.getX(), blueValue.getY()));
    }

    public static Pose2d MirroredPose2d(Pose2d blueValue){
        return new Pose2d(MirroredTranslation2d(blueValue.getTranslation()), MirroredRotation2d(blueValue.getRotation()));
    }

    public static Translation3d MirroredTranslation3d(Translation3d blueValue){
        Translation2d temp = MirroredTranslation2d(blueValue.toTranslation2d());
        return new Translation3d(temp.getX(), temp.getY(), blueValue.getZ());
    }

    public static Pose3d MirroredPose3d(Pose3d blueValue){
        return new Pose3d(MirroredTranslation3d(blueValue.getTranslation()), MirroredRotation3d(blueValue.getRotation()));
    } 

    public static RectanglePoseArea PoseArea(RectanglePoseArea blueValue){
        return new RectanglePoseArea(
            Translation2d(
                blueValue.getBottomLeftPoint(), 
                new Translation2d(blueValue.getMinX() + FieldConstants.FIELD_DIMENSIONS.getX()/2.0, blueValue.getMinY())),
            Translation2d(
                blueValue.getTopRightPoint(), 
                new Translation2d(blueValue.getMaxX() + FieldConstants.FIELD_DIMENSIONS.getX()/2.0, blueValue.getMaxY())));
    }
}
