package frc.robot;


import com.spikes2212.util.controllers.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.swerve.Swerve;

public class OI {

    private final PlaystationControllerWrapper playstation;

    public OI() {
        playstation = new PlaystationControllerWrapper(0);
        playstation.getCircleButton().onTrue(new InstantCommand(() -> Swerve.getInstance().resetFieldRelativity()));
    }

    public double getRightX() {
        double input = playstation.getRightX();
        if (Math.abs(input) < 0.08) return 0;
        return input;
    }

    public double getLeftX() {
        double input = playstation.getLeftX();
        if (Math.abs(input) < 0.08) return 0;
        return input;
    }

    public double getLeftY() {
        double input = playstation.getLeftY();
        if (Math.abs(input) < 0.08) return 0;
        return input;
    }
}
