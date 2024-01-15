package frc.lib.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.math.Conversions;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;

public final class AllianceFlippable {

    private static boolean flip = RobotContainer.getAlliance() == Alliance.Blue;

    public static Rotation2d ForwardRotation(){
        return flip ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    public static double Number(double blueValue, double redValue){
        return flip ? blueValue : redValue;
    }

    public static Rotation2d Rotation2d(Rotation2d blueValue, Rotation2d redValue){
        return flip ? blueValue : redValue;
    }

    public static Rotation2d Rotation2d(Rotation2d blueValue){
        return blueValue.rotateBy(Rotation2d.fromDegrees(180));
    }

    public static Translation2d Translation2d(Translation2d blueValue, Translation2d redValue){
        return flip ? blueValue : redValue;
    }

    public static Translation2d MapTranslation2d(Translation2d blueValue){
        double temp = Conversions.mapRange(blueValue.getX(), 0.0, FieldConstants.FIELD_DIMENSIONS.getX()/2, FieldConstants.FIELD_DIMENSIONS.getX(), FieldConstants.FIELD_DIMENSIONS.getX()/2);
        return new Translation2d(temp, blueValue.getY());
    }

    public static Pose2d Pose2d(Pose2d blueValue, Pose2d redValue){
        return flip ? blueValue : redValue;
    }

    public static Pose2d MapPose2d(Pose2d blueValue){
        return new Pose2d(MapTranslation2d(blueValue.getTranslation()), ForwardRotation());
    }

    public static Translation3d MapTranslation3d(Translation3d blueValue){
        Translation2d temp = MapTranslation2d(blueValue.toTranslation2d());
        return new Translation3d(temp.getX(), temp.getY(), blueValue.getZ());
    }
}
