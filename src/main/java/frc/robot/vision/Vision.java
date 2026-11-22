package frc.robot.vision;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

import java.util.Optional;

public class Vision extends SubsystemBase {

    private final Pigeon2 pigeon2 = new Pigeon2(RobotMap.CAN.PIGEON2_ID,
            new CANBus("canivore"));

    private final Limelight limelight;
    private final RootNamespace namespace = new RootNamespace(VisionConstants.NAMESPACE_NAME);
    private final LimelightPoseEstimator poseEstimator;
    private Pose3d pos = Pose3d.kZero;

//    private final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(
//            List.of(new AprilTag(4, new Pose3d()), new AprilTag(2, new Pose3d(0, 0.72, -0.22, Rotation3d.kZero))),
//            17.5483, 8.0519016);

    public Vision() {
        this.limelight = new Limelight(VisionConstants.CAMERA_NAME);
        limelight.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(VisionConstants.OFFSET)
                .save();

        this.poseEstimator = new LimelightPoseEstimator(limelight, LimelightPoseEstimator.EstimationMode.MEGATAG2);
        configureNamespace();
    }

    public void configureNamespace() {
        namespace.publishDouble("Horizontal Offset", this::getHorizontalOffset);
        namespace.publishDouble("Vertical Offset", this::getVerticalOffset);
        namespace.publishDouble("Distance from target", this::getDistanceFromTag);
        namespace.publishBoolean("target", () -> limelight.getData().targetData.getTargetStatus());
        namespace.publishDouble("temperature", this::getTemperature);

        namespace.publishDouble("x", () -> getPose().getX());
        namespace.publishDouble("y", () -> getPose().getY());
        namespace.publishDouble("z", () -> getPose().getZ());
    }

    public Pose3d getPose() {
        return pos;
    }

    public double getHorizontalOffset() {
        return limelight.getData().targetData.getHorizontalOffset();
    }

    public double getTemperature() {
        double[] arr = limelight.getNTTable().getEntry("hw").getDoubleArray(new double[0]);
        return arr.length > 0 ? arr[0] : -1;
    }

//    public void updatePose() {
//        double[] arr = limelight.getNTTable().getEntry("botpose").getDoubleArray(new double[0]);
//        pos =  arr.length > 0 ? new Pose3d(arr[0], arr[1], arr[2], Rotation3d.kZero) : new Pose3d();
//    }

    public double getVerticalOffset() {
        return limelight.getData().targetData.getVerticalOffset();
    }

    public void setThrottle(boolean enable) {
        limelight.getSettings().withThrottle(enable ? VisionConstants.THROTTLE : 0).save();
    }

    public double getCurrentTagID() {
        return limelight.getData().targetData.getAprilTagID();
    }

    public double getDistanceFromTag() {
        var data = limelight.getData().getRawFiducials();
        double targetID = getCurrentTagID();

        for (RawFiducial tag : data) {
            if (tag.id == targetID) {
                return tag.distToCamera;
            }
        }
        return -1.0;
    }

    public void updatePose() {
        Optional<PoseEstimate> poseEstimate = poseEstimator.getPoseEstimate();
        if (poseEstimate.isEmpty()) return;

        if (isPoseEstimateAccurate(poseEstimate.get())) {
            pos = poseEstimate.get().pose;
        }
    }

    public boolean isPoseEstimateAccurate(PoseEstimate poseEstimate) {
        return poseEstimate.avgTagDist < 4 && poseEstimate.tagCount > 0;
    }

    @Override
    public void periodic() {
        namespace.update();
        updateRotation();
        updatePose();
    }

    public void updateRotation() {
        limelight.getSettings().withRobotOrientation(
                new Orientation3d(new Rotation3d(pigeon2.getRotation2d()), Units.DegreesPerSecond.of(0),
                        Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))
        ).save();
    }
}