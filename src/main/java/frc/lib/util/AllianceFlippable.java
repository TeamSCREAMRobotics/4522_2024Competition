package frc.lib.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.math.Conversions;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;

public final class AllianceFlippable {

    private static boolean flip = RobotContainer.getAlliance() == Alliance.Blue;

    public static Rotation2d getForwardRotation(){
        return flip ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    public static int getDirectionCoefficient(){
        return flip ? 1 : -1;
    }

    public static Translation2d getTargetSpeaker(){
        return Translation2d(FieldConstants.BLUE_SPEAKER_OPENING, FieldConstants.RED_SPEAKER_OPENING);
    }

    public static Object Object(Object blueValue, Object redValue){
        return flip ? blueValue : redValue;
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
        double temp = Conversions.mapRange(blueValue.getX(), 0.0, FieldConstants.FIELD_DIMENSIONS.getX()/2, FieldConstants.FIELD_DIMENSIONS.getX(), FieldConstants.FIELD_DIMENSIONS.getX()/2);
        return new Translation2d(temp, blueValue.getY());
    }

    public static Pose2d MirroredPose2d(Pose2d blueValue){
        return new Pose2d(MirroredTranslation2d(blueValue.getTranslation()), getForwardRotation());
    }

    public static Translation3d MirroredTranslation3d(Translation3d blueValue){
        Translation2d temp = MirroredTranslation2d(blueValue.toTranslation2d());
        return new Translation3d(temp.getX(), temp.getY(), blueValue.getZ());
    }

    public static Pose3d MirroredPose3d(Pose3d blueValue){
        return new Pose3d(MirroredTranslation3d(blueValue.getTranslation()), MirroredRotation3d(blueValue.getRotation()));
    } 
}
