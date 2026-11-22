package frc.robot.vision;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

import java.util.Optional;

public class Vision extends SubsystemBase {

    private final RootNamespace namespace;
    private final Limelight limelight;
    private final LimelightPoseEstimator poseEstimator;
    private Pose3d robotPose = new Pose3d();

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        this.limelight = new Limelight(VisionConstants.LIMELIGHT_NAME);
        this.poseEstimator = limelight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);
        this.namespace = new RootNamespace(VisionConstants.NAMESPACE_NAME);
        configureLimelight(limelight);
        configureDashboard();
    }

    private void configureLimelight(Limelight limelight) {
        limelight.getSettings()
                .withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
                .withCameraOffset(VisionConstants.CAMERA_OFFSET)
                .save();
    }

    public void configureDashboard() {
        namespace.publishDouble("tag id", this::getCurrentTagId);
        namespace.publishDouble("temperature", this::getTemperature);
        namespace.publishDouble("horizontal offset", this::getHorizontalOffset);
        namespace.publishBoolean("seeing target", this::seeingTarget);
        namespace.publishDouble("vertical offset", this::getVerticalOffset);
        namespace.publishDouble("x", () -> robotPose.getX());
        namespace.publishDouble("y", () -> robotPose.getY());
        namespace.publishDouble("z", () -> robotPose.getZ());
    }

    public boolean seeingTarget() {
        return limelight.getData().targetData.getTargetStatus();
    }

    public double getHorizontalOffset() {
        return limelight.getData().targetData.getHorizontalOffset();
    }

    public double getVerticalOffset() {
        return limelight.getData().targetData.getVerticalOffset();
    }

    public double getTemperature() {
        double[] hw = limelight.getNTTable().getEntry("hw").getDoubleArray(new double[0]);
        return hw.length > 0 ? hw[0] : -1;
    }

    public double getCurrentTagId() {
        return limelight.getData().targetData.getAprilTagID();
    }

    public double getDistanceFromTag() {
        var data = limelight.getData();
        double targetId = getCurrentTagId();

        for (RawFiducial tag : data.getRawFiducials()) {
            if (tag.id == targetId) {
                return tag.distToCamera;
            }
        }
        return -1;
    }

    public double getAverageDistanceFromTag() {
        double[] botpose = limelight.getNTTable().getEntry("botpose").getDoubleArray(new double[0]);
        return botpose.length > 0 ? botpose[9] : -1;
    }

    public void setThrottle(boolean enable) {
        limelight.getSettings().withThrottle(enable ? VisionConstants.THROTTLE : 0).save();
    }

    public void updatePose() {
        Optional<PoseEstimate> optional = poseEstimator.getPoseEstimate();
        if (optional.isEmpty()) return;

        PoseEstimate poseEstimate = optional.get();
        if (!isMeasurementAccurate(poseEstimate)) return;

        robotPose = poseEstimate.pose;
    }

    private boolean isMeasurementAccurate(PoseEstimate poseEstimate) {
        return poseEstimate.avgTagDist < 4 && poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.3;
    }

    @Override
    public void periodic() {
        namespace.update();
        updatePose();
    }
}
