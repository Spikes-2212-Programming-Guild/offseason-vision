package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Shooter;

public class ShootWithDistance extends SmartShoot {

    private static InterpolatingDoubleTreeMap treeMap = getTreeMap();

    private static InterpolatingDoubleTreeMap getTreeMap() {
        if (ShootWithDistance.treeMap == null) {
            treeMap = new InterpolatingDoubleTreeMap();
        }

        return ShootWithDistance.treeMap;
    }

    public ShootWithDistance(Shooter shooter, double distance) {
        super(shooter, () -> getTargetVel(distance));


    }

    public static double getTargetVel(double distance) {
        return getTreeMap().get(distance);
    }
}
