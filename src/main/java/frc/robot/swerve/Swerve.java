package frc.robot.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.commandbased.swerve.SwerveDrivetrain;
import com.spikes2212.commandbased.swerve.SwerveModule;
import com.spikes2212.commandbased.swerve.commands.DriveSwerve;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionMeasurement;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.swerve.SwerveConstants.DRIVETRAIN.*;

public class Swerve extends SwerveDrivetrain {

    private final Pigeon2 gyro;
    private final Field2d field2d;
    private final SwerveDrivePoseEstimator poseEstimator;

    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID, CAN_BUS));
        }
        return instance;
    }

    private Swerve(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                   SwerveModule backLeftModule, SwerveModule backRightModule,
                   double drivetrainTrackWidth, double drivetrainTrackLength,
                   double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;
        field2d = new Field2d();
        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(), getAngle(), getSwerveModulePositions(), new Pose2d()
        );
        configureDashboard();
    }

    @Override
    public Rotation2d getAngle() {
        return gyro.getRotation2d();
    }

    @Override
    public void resetAngleSensor() {
        setGyro(GYRO_OFFSET);
    }

    public void setGyro(double angle) {
        gyro.setYaw(angle);
    }

    @Override
    public void configureDashboard() {
        namespace.publishSendable("field", field2d);
        namespace.publishDouble("angle", () -> getAngle().getDegrees());
        namespace.publishRunnable("reset gyro", this::resetAngleSensor);
    }

    @Override
    public void periodic() {
        super.periodic();
        updateOdometry();
        updateVision();
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public void updateVision() {
        Vision.getInstance().updateYaw(this.getAngle().getRadians());
        Optional<VisionMeasurement> optional = Vision.getInstance().getVisionMeasurement();
        if (optional.isPresent()) {
            addVisionMeseur(optional.get());
        }
    }

    public void addVisionMeseur(VisionMeasurement visionMeasurement) {
        poseEstimator.addVisionMeasurement(
                visionMeasurement.poseEstimate().pose.toPose2d(),
                visionMeasurement.poseEstimate().timestampSeconds,
                visionMeasurement.standardDeviations().toMatrix()
        );
    }

    public void updateOdometry() {
        poseEstimator.update(getAngle(), getSwerveModulePositions());
    }

    /**
     * converts controller coordinate system to wpilib coordinate system (blue/red depends on the game)
     */
    public Command driveWithController(Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightX,
                                       boolean isFieldRelative, double speedMultiplier) {
        double allianceMultiplier = isFieldRelative && DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 1.0 : -1.0;

        return new DriveSwerve(
                this,
                () -> -leftY.get() * allianceMultiplier * speedMultiplier,
                () -> -leftX.get() * allianceMultiplier * speedMultiplier,
                () -> -rightX.get() * speedMultiplier, isFieldRelative
        );
    }
}