package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.commandbased.swerve.SwerveModule;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import static frc.robot.swerve.SwerveConstants.MODULE.*;

public class SwerveModuleRebuilt extends SwerveModule {

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;

    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted,
                               double absoluteEncoderOffset, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffset,
                DRIVE_PID_SETTINGS, TURN_PID_SETTINGS, DRIVE_FF_SETTINGS, TURN_FF_SETTINGS, MIN_SPEED_LIMIT,
                UPDATE_DRIVE_PERIODICALLY, UPDATE_TURN_PERIODICALLY);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    @Override
    protected void configureDriveController() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(NeutralModeValue.Brake);
        driveMotor.setEncoderConversionFactor(DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS);
        driveMotor.configureLoop(driveMotorPIDSettings, driveMotorFFSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.restoreFactoryDefaults();
        turnMotor.setInverted(turnMotorInverted);
        turnMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        turnMotor.setPositionConversionFactor(TURN_POSITION_IN_ROTATION);
        turnMotor.setVelocityConversionFactor(TURN_VELOCITY_IN_ROTATION);
        turnMotor.configureLoop(turnMotorPIDSettings, turnMotorFFSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().
                withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT).
                withSensorDirection(turnMotorInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(absoluteEncoderOffset / DEGREES_IN_ROTATION);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degree));
    }

    @Override
    public void configureDashboard() {
        namespace.publishDouble("absolute encoder", () -> this.getAbsoluteModuleAngle().getDegrees());
        namespace.publishDouble("relative angle", this::getRelativeModuleAngle);

        Supplier<Double> targetAngle = namespace.addTunableDouble("target angle", 0);
        namespace.publishCommand("set target angle", Commands.runEnd(
                () -> setTargetAngle(Rotation2d.fromDegrees(targetAngle.get())),
                this::stop,
                this
        ));

        namespace.publishDouble("velocity", this::getDrivingVelocity);
        Supplier<Double> targetVelocity = namespace.addTunableDouble("target velocity", 0);
        namespace.publishCommand("set target velocity", Commands.runEnd(
                () -> setTargetVelocity(targetVelocity.get()),
                this::stop,
                this
        ));
    }
}
