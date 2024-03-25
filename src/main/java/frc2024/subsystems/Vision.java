package frc2024.subsystems;

import java.nio.channels.Pipe;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.PhotonUtils;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.LimelightHelpers;
import com.team4522.lib.util.RunOnce;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.FieldConstants;
import frc2024.Constants.VisionConstants;

public class Vision{

    private static final LinearFilter txFilter = LinearFilter.movingAverage(8);
    private static final LinearFilter tyFilter = LinearFilter.movingAverage(8);
    private static final LinearFilter taFilter = LinearFilter.movingAverage(8);
    private static final LinearFilter distanceFilter = LinearFilter.movingAverage(5);

    private static final RunOnce filterReset = new RunOnce();

    public enum Limelight{
        TRAP("limelight-trap", new Pose3d()), 
        SHOOT_SIDE("limelight-shooter", new Pose3d(0.286, -0.162, 0.233, new Rotation3d(0, Math.toRadians(27), Math.toRadians(180.0)))), // z: 0.220615
        INTAKE_SIDE("limelight-intake", new Pose3d());

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
        return txFilter.calculate(LimelightHelpers.getTX(limelight.name));
    }

    public static double getTY(Limelight limelight){
        return tyFilter.calculate(LimelightHelpers.getTY(limelight.name));
    }

    public static double getTA(Limelight limelight){
        return taFilter.calculate(LimelightHelpers.getTA(limelight.name));
    }

    public static boolean getTV(Limelight limelight){
        return LimelightHelpers.getTV(limelight.name);
    }

    public static double getLatency_Pipeline(Limelight limelight){
        return LimelightHelpers.getLatency_Pipeline(limelight.name);
    }

    public static double getLatency_Capture(Limelight limelight){
        return LimelightHelpers.getLatency_Capture(limelight.name);
    }

    public static Pose2d getBotPose2d(Limelight limelight){
        return LimelightHelpers.getBotPose2d_wpiBlue(limelight.name);
    }

    public static Pose3d getBotPose3d(Limelight limelight){
        return LimelightHelpers.getBotPose3d_wpiBlue(limelight.name);
    }

    public static Pose2d getBotPose2d_TargetSpace(Limelight limelight){
        return LimelightHelpers.getBotPose3d_TargetSpace(limelight.name).toPose2d();
    }

    public static Pose3d getBotPose3d_TargetSpace(Limelight limelight){
        return LimelightHelpers.getBotPose3d_TargetSpace(limelight.name);
    }

    public static double getDistanceToTargetMeters(double targetHeight, Limelight limelight){
        double goal_theta = limelight.mountPose.getRotation().getY() + Math.toRadians(getTY(Limelight.SHOOT_SIDE));
        double height_diff = targetHeight - limelight.mountPose.getZ();

        return height_diff / Math.tan(goal_theta);
    }

    public static double getFusedDistanceToSpeaker(Limelight limelight, Pose2d currentPose){
        double sum;
        sum = ScreamUtil.calculateDistanceToTranslation(currentPose.getTranslation(), AllianceFlipUtil.getTargetSpeaker().getTranslation());
        if(getTV(limelight)){
            sum += getDistanceToTargetMeters(FieldConstants.SPEAKER_TAG_HEIGHT, limelight);
            return sum / 2;
        }
        return sum;
    }

    public static boolean getLockedToTarget(Limelight limelight){
        return Math.abs(getTX(limelight)) <= VisionConstants.AUTO_FIRE_X_THRESHOLD;
    }

    public static Optional<TimestampedVisionMeasurement> getTimestampedVisionMeasurement(Limelight limelight){
        if(LimelightHelpers.getBotPose(limelight.name).length == 0){
            return Optional.empty();
        }
        return Optional.of(new TimestampedVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name).pose, Timer.getFPGATimestamp() - LimelightHelpers.getBotPose(limelight.name)[6]));
    }

    /* public static Optional<TimestampedVisionMeasurement>[] getBotPoses(){
        return new Optional<TimestampedVisionMeasurement>[] {
            getTimestampedVisionMeasurement(Limelight.SHOOTER)
        };
    } */

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

    public static void setPriorityTagID(int id, Limelight limelight){
        LimelightHelpers.setPriorityTagID(limelight.name, id);
    }

    public static void setPipeline(Limelight limelight, int index){
        LimelightHelpers.setPipelineIndex(limelight.name, index);
    }

    public static void setPipeline(IntakePipeline pipeline){
        setPipeline(Limelight.INTAKE_SIDE, pipeline.index);
    }

    public static void setPipeline(FrontPipeline pipeline){
        setPipeline(Limelight.SHOOT_SIDE, pipeline.index);
    }

    public static void setPipeline(BackPipeline pipeline){
        setPipeline(Limelight.TRAP, pipeline.index);
    }

    public static void periodic() {
        if(!getTV(Limelight.SHOOT_SIDE)){
            filterReset.runOnce(
                () -> {
                    txFilter.reset();
                    tyFilter.reset();
                    taFilter.reset();
                    distanceFilter.reset();
                });
        } else {
            filterReset.reset();
        }
        // System.out.println(Units.metersToInches(getDistanceToTargetMeters(FieldConstants.SPEAKER_TAG_HEIGHT, Limelight.SHOOTER)));
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
