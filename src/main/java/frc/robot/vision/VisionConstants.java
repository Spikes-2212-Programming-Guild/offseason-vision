package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class VisionConstants {
    public static final String NAMESPACE_NAME = "vision";
    public static final String CAMERA_NAME = "limelight";
    public static final double THROTTLE = 200.0;
    public static final Pose3d OFFSET = new Pose3d(30, 9, 17, new Rotation3d(0, 45, 0));
}
