package frc.robot;


import com.spikes2212.util.controllers.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.swerve.Swerve;

public class OI {

    private final PlaystationControllerWrapper playstation;

    public OI() {
        playstation = new PlaystationControllerWrapper(0);
        playstation.getCircleButton().onTrue(new InstantCommand(() -> Swerve.getInstance().resetFieldRelativity()));
    }

    public double getRightX() {
        double rightX = playstation.getRightX();
        double sign = Math.signum(rightX);
        double input = Math.pow(rightX, 2);
        if (input < 0.08) return 0;
        return input * sign;
    }

    public double getLeftX() {
        double leftX = playstation.getLeftX();
        double sign = Math.signum(leftX);
        double input = Math.pow(leftX, 2);
        if (input < 0.08) return 0;
        return input * sign;
    }

    public double getLeftY() {
        double leftY = playstation.getLeftY();
        double sign = Math.signum(leftY);
        double input = Math.pow(leftY, 2);
        if (input < 0.08) return 0;
        return input * sign;
    }
}