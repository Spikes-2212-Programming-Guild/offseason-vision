package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.spikes2212.control.FFController;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;

public class SwerveConstants {

    public static final RootNamespace namespace = new RootNamespace("module tuner");

    interface MODULE {
        double MIN_SPEED_LIMIT = 0.1;
        double DRIVE_GEAR_RATIO = 1 / 6.12;
        double TURN_GEAR_RATIO = (double) 7 / 150;
        double WHEEL_DIAMETER_METERS = 0.1016;
        int DEGREES_IN_ROTATION = 360;
        int SECONDS_IN_MINUTE = 60;
        int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

        PIDSettings DRIVE_PID_SETTINGS = namespace.addPIDNamespace("drive", new PIDSettings(
                0.015, 0, 0, 0, 0, 0
        ));

        FFSettings DRIVE_FF_SETTINGS = namespace.addFFNamespace("drive", new FFSettings(
                0.014, 0.175, 0, FFController.ControlMode.LINEAR_VELOCITY
        ));

        PIDSettings TURN_PID_SETTINGS = namespace.addPIDNamespace("turn", new PIDSettings(
                0.012, 0, 0.0001, 0, 0, 0
        ));

        FFSettings TURN_FF_SETTINGS = namespace.addFFNamespace("turn", new FFSettings(
                FFController.ControlMode.LINEAR_POSITION
        ));

        boolean UPDATE_TURN_PERIODICALLY = false;
        boolean UPDATE_DRIVE_PERIODICALLY = false;
    }

    interface DRIVETRAIN {
        String NAMESPACE_NAME = "swerve";
        CANBus CAN_BUS = new CANBus("canivore");
        double MAX_POSSIBLE_VELOCITY = 5.1;
        double TRACK_WIDTH = 0.55;
        double TRACK_LENGTH = 0.55;
        int GYRO_OFFSET = 180;
    }

    interface MODULE_HOLDER {
        CANBus CAN_BUS = new CANBus("canivore");

        boolean FRONT_LEFT_DRIVE_INVERTED = false;
        boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        boolean BACK_LEFT_DRIVE_INVERTED = false;
        boolean BACK_RIGHT_DRIVE_INVERTED = true;

        boolean FRONT_LEFT_CANCODER_INVERTED = false;
        boolean FRONT_RIGHT_CANCODER_INVERTED = false;
        boolean BACK_LEFT_CANCODER_INVERTED = false;
        boolean BACK_RIGHT_CANCODER_INVERTED = false;

        String FRONT_LEFT_NAMESPACE_NAME = "front left";
        String FRONT_RIGHT_NAMESPACE_NAME = "front right";
        String BACK_LEFT_NAMESPACE_NAME = "back left";
        String BACK_RIGHT_NAMESPACE_NAME = "back right";

        double FRONT_LEFT_OFFSET = 61.640;
        double FRONT_RIGHT_OFFSET = 27.509;
        double BACK_LEFT_OFFSET = 162.105;
        double BACK_RIGHT_OFFSET = 168.041;
    }

    public static void updateNamespace() {
        namespace.update();
    }
}
