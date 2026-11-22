package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static final String LIMELIGHT_NAME = "limelight";
    public static final String NAMESPACE_NAME = "vision";
    public static final int THROTTLE = 200;
    public static final Pose3d CAMERA_OFFSET = new Pose3d(
            new Translation3d(0.3, 0.09, 0.16),
            new Rotation3d(Units.degreesToRadians(-5), Units.degreesToRadians(50), 0)
    );
}
