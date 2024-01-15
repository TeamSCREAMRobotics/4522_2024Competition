package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.util.LimelightHelpers;

public class Vision {

    private static LinearFilter filter = LinearFilter.movingAverage(25);

    public record CropWindow(double cropXMin, double cropXMax, double cropYMin, double cropYMax){};
    
    public enum Limelight{
        BACK("limelight-back"), FRONT("limelight-front"), INTAKE("limelight-intake");

        String name;

        private Limelight(String name){
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

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

    public static Pose2d getBotPose2d(Limelight limelight){
        return LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName());
    }

    public static Pose3d getBotPose3d(Limelight limelight){
        return LimelightHelpers.getBotPose3d_wpiBlue(limelight.getName());
    }

    public static Pose2d getBosePose2d_TargetSpace(Limelight limelight){
        return LimelightHelpers.getBotPose3d_TargetSpace(limelight.getName()).toPose2d();
    }

    public static Pose3d getBosePose3d_TargetSpace(Limelight limelight){
        return LimelightHelpers.getBotPose3d_TargetSpace(limelight.getName());
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

    public static void setCropWindow(Limelight limelight, CropWindow window){
        LimelightHelpers.setCropWindow(limelight.getName(), window.cropXMin(), window.cropXMax(), window.cropYMin(), window.cropYMax());
    }

    public static void setPipeline(Limelight limelight, int index){
        LimelightHelpers.setPipelineIndex(limelight.getName(), index);
    }

    public static double filter(double value){
        return filter.calculate(value);
    }
}
