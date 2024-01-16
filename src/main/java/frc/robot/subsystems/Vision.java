package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision {

    private static LinearFilter numberFilter = LinearFilter.movingAverage(25);
    private static LinearFilter poseFilter = LinearFilter.movingAverage(10);
    
    public enum Limelight{
        BACK("limelight-back", new Pose3d()), FRONT("limelight-front", new Pose3d()), INTAKE("limelight-intake", new Pose3d());

        String name;
        Pose3d mountPose;

        private Limelight(String name, Pose3d mountPose){
            this.name = name;
            this.mountPose = mountPose;
        }

        public String getName() {
            return name;
        }

        public Pose3d getMountPose() {
            return mountPose;
        }
    }

    public record TimestampedVisionMeasurement(Pose2d pose, double timestamp){}

    public enum LEDMode{
        OFF, ON, BLINK;
    }

    public static double getTX(Limelight limelight){
        return filter(LimelightHelpers.getTX(limelight.getName()));
    }

    public static double getTY(Limelight limelight){
        return filter(LimelightHelpers.getTY(limelight.getName()));
    }

    public static double getTA(Limelight limelight){
        return filter(LimelightHelpers.getTA(limelight.getName()));
    }

    public static boolean getTV(Limelight limelight){
        return LimelightHelpers.getTV(limelight.getName());
    }

    public static double getLatency(Limelight limelight){
        return LimelightHelpers.getLatency_Pipeline(limelight.getName());
    }

    public static Pose2d getBotPose2d(Limelight limelight){
        return filterPose2d(LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName()));
    }

    public static Pose3d getBotPose3d(Limelight limelight){
        return filterPose3d(LimelightHelpers.getBotPose3d_wpiBlue(limelight.getName()));
    }

    public static Pose2d getBotPose2d_TargetSpace(Limelight limelight){
        return filterPose2d(LimelightHelpers.getBotPose3d_TargetSpace(limelight.getName()).toPose2d());
    }

    public static Pose3d getBotPose3d_TargetSpace(Limelight limelight){
        return filterPose3d(LimelightHelpers.getBotPose3d_TargetSpace(limelight.getName()));
    }

    public static double getDistanceFromSpeaker(Limelight limelight){
        Rotation2d angleToGoal = Rotation2d.fromRadians(limelight.getMountPose().getRotation().getY()).plus(Rotation2d.fromDegrees(getTY(limelight)));
        return (FieldConstants.SPEAKER_TAGS_HEIGHT - limelight.getMountPose().getZ()) / angleToGoal.getTan();
    }

    public static TimestampedVisionMeasurement getTimestampedVisionMeasurement(Limelight limelight){
        return new TimestampedVisionMeasurement(getBotPose2d(limelight), Timer.getFPGATimestamp() - LimelightHelpers.getBotPose(limelight.getName())[6]/1000.0);
    }

    public static TimestampedVisionMeasurement[] getBotPoses(){
        return new TimestampedVisionMeasurement[] {
            getTimestampedVisionMeasurement(Limelight.FRONT),
            getTimestampedVisionMeasurement(Limelight.BACK)
        };
    }

    public static int getCurrentPipeline(Limelight limelight){
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelight.getName());
    }

    public static void setLEDMode(Limelight limelight, LEDMode ledMode){
        switch (ledMode) {
            case OFF:
            LimelightHelpers.setLEDMode_ForceOff(limelight.getName());
            break;
            case ON:
            LimelightHelpers.setLEDMode_ForceOn(limelight.getName());
            break;
            case BLINK:
            LimelightHelpers.setLEDMode_ForceBlink(limelight.getName());
            break;
        }
    }

    public static void setPipeline(Limelight limelight, int index){
        LimelightHelpers.setPipelineIndex(limelight.getName(), index);
    }

    public static double filter(double value){
        return numberFilter.calculate(value);
    }

    public static Pose2d filterPose2d(Pose2d pose){
        return new Pose2d(
            poseFilter.calculate(pose.getX()),
            poseFilter.calculate(pose.getY()),
            Rotation2d.fromRadians(poseFilter.calculate(pose.getRotation().getRadians()))
        );
    }

    public static Pose3d filterPose3d(Pose3d pose){
        return new Pose3d(
            poseFilter.calculate(pose.getX()),
            poseFilter.calculate(pose.getY()),
            poseFilter.calculate(pose.getZ()),
            new Rotation3d(
               poseFilter.calculate(pose.getRotation().getX()), 
               poseFilter.calculate(pose.getRotation().getY()), 
               poseFilter.calculate(pose.getRotation().getZ())
            ));
    }
}
