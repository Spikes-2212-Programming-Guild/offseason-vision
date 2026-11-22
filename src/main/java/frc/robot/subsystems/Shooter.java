package frc.robot.subsystems;

import com.spikes2212.commandbased.subsystems.smart.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends SmartMotorControllerGenericSubsystem {

    private final TalonFXWrapper motor;
    private static Shooter instance;

    private Shooter(TalonFXWrapper motor) {
        super("shooter", motor);
        this.motor = motor;
        motor.setInverted(true);
        configureDashboard();
    }

    public static Shooter getInstance() {
        if (Shooter.instance == null)
            Shooter.instance = new Shooter(new TalonFXWrapper(RobotMap.CAN.SHOOTER_ID));
        return instance;
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void configureDashboard() {
        namespace.publishDouble("velocity", motor::getVelocity);
    }
}
