package frc.robot.commands;

import com.spikes2212.commandbased.subsystems.smart.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.FFController;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class SmartShoot extends MoveSmartMotorControllerGenericSubsystem {

    private final static RootNamespace namespace = new RootNamespace("smart shoot");
    private final static PIDSettings pidSettings = namespace.addPIDNamespace("pid",
            new PIDSettings(0, 0, 0, 0, 0, 0));
    private final static FFSettings feedForwardSettings = namespace.addFFNamespace("feed forward",
            new FFSettings(0, 0, 0, FFController.ControlMode.LINEAR_VELOCITY));

    public SmartShoot(Shooter shooter, Supplier<Double> setpoint) {
        super(shooter, pidSettings, feedForwardSettings, UnifiedControlMode.VELOCITY, setpoint, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static void updateNamespace() {
        namespace.update();
    }
}
