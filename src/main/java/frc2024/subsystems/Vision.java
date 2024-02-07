package frc2024.subsystems;

import java.nio.channels.Pipe;

import org.photonvision.PhotonUtils;

import com.team4522.lib.util.LimelightHelpers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.VisionConstants;

public class Vision {

    private static LinearFilter numberFilter = LinearFilter.movingAverage(5);
    private static LinearFilter poseFilter = LinearFilter.movingAverage(5);
    
    public enum Limelight{
        TRAP("limelight-back", new Pose3d()), SHOOTER("limelight-front", new Pose3d()), INTAKE("limelight-intake", new Pose3d());

        String name;
        Pose3d mountPose;

        private Limelight(String name, Pose3d mountPose){
            this.name = name;
            this.mountPose = mountPose;
        }
    }

    public enum IntakePipeline{
        DETECTOR_LARGEST(0), DETECTOR_LEFTMOST(1), DETECTOR_RIGHTMOST(2);

        int index;

        private IntakePipeline(int index){
            this.index = index;
        }
    }

    public enum FrontPipeline{
        FIDUCIAL(0);

        int index;

        private FrontPipeline(int index){
            this.index = index;
        }
    }

    public enum BackPipeline{
        FIDUCIAL(0);

        int index;

        private BackPipeline(int index){
            this.index = index;
        }
    }

    public record TimestampedVisionMeasurement(Pose2d pose, double timestamp){}

    public enum LEDMode{
        OFF, ON, BLINK;
    }

    public static double getTX(Limelight limelight){
        return filter(LimelightHelpers.getTX(limelight.name));
    }

    public static double getTY(Limelight limelight){
        return filter(LimelightHelpers.getTY(limelight.name));
    }

    public static double getTA(Limelight limelight){
        return filter(LimelightHelpers.getTA(limelight.name));
    }

    public static boolean getTV(Limelight limelight){
        return LimelightHelpers.getTV(limelight.name);
    }

    public static double getLatency(Limelight limelight){
        return LimelightHelpers.getLatency_Pipeline(limelight.name);
    }

    public static Pose2d getBotPose2d(Limelight limelight){
        return filterPose2d(LimelightHelpers.getBotPose2d_wpiBlue(limelight.name));
    }

    public static Pose3d getBotPose3d(Limelight limelight){
        return filterPose3d(LimelightHelpers.getBotPose3d_wpiBlue(limelight.name));
    }

    public static Pose2d getBotPose2d_TargetSpace(Limelight limelight){
        return filterPose2d(LimelightHelpers.getBotPose3d_TargetSpace(limelight.name).toPose2d());
    }

    public static Pose3d getBotPose3d_TargetSpace(Limelight limelight){
        return filterPose3d(LimelightHelpers.getBotPose3d_TargetSpace(limelight.name));
    }

    public static double getDistanceFromSpeaker(Limelight limelight){
        return PhotonUtils.calculateDistanceToTargetMeters(limelight.mountPose.getZ(), FieldConstants.SPEAKER_TAGS_HEIGHT, limelight.mountPose.getRotation().getY(), Units.degreesToRadians(getTY(limelight)));
    }

    public static TimestampedVisionMeasurement getTimestampedVisionMeasurement(Limelight limelight){
        return new TimestampedVisionMeasurement(getBotPose2d(limelight), Timer.getFPGATimestamp() - LimelightHelpers.getBotPose(limelight.name)[6]/1000.0);
    }

    public static TimestampedVisionMeasurement[] getBotPoses(){
        return new TimestampedVisionMeasurement[] {
            getTimestampedVisionMeasurement(Limelight.SHOOTER),
            getTimestampedVisionMeasurement(Limelight.TRAP)
        };
    }

    public static int getCurrentPipeline(Limelight limelight){
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelight.name);
    }

    public static void setLEDMode(Limelight limelight, LEDMode ledMode){
        switch (ledMode) {
            case OFF:
            LimelightHelpers.setLEDMode_ForceOff(limelight.name);
            break;
            case ON:
            LimelightHelpers.setLEDMode_ForceOn(limelight.name);
            break;
            case BLINK:
            LimelightHelpers.setLEDMode_ForceBlink(limelight.name);
            break;
        }
    }

    public static void setPipeline(Limelight limelight, int index){
        LimelightHelpers.setPipelineIndex(limelight.name, index);
    }

    public static void setPipeline(IntakePipeline pipeline){
        setPipeline(Limelight.INTAKE, pipeline.index);
    }

    public static void setPipeline(FrontPipeline pipeline){
        setPipeline(Limelight.SHOOTER, pipeline.index);
    }

    public static void setPipeline(BackPipeline pipeline){
        setPipeline(Limelight.TRAP, pipeline.index);
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

    public Command intakePipelineCommand(IntakePipeline pipeline){
        return Commands.runOnce(() -> setPipeline(pipeline));
    }
    
    public Command frontPipelineCommand(FrontPipeline pipeline){
        return Commands.runOnce(() -> setPipeline(pipeline));
    }

    public Command backPipelineCommand(BackPipeline pipeline){
        return Commands.runOnce(() -> setPipeline(pipeline));
    }
}
