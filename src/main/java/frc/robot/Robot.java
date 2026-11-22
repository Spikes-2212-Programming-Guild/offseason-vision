// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.Vision;

public class Robot extends TimedRobot {

    private final Vision vision;
//    private final RootNamespace namespace = new RootNamespace("robot");

    public Robot() {
        vision = new Vision();
//        Supplier<Double> setPoint = namespace.addTunableDouble("setpoint", 0.0);
//        Supplier<Double> speed = namespace.addTunableDouble("speed", 0.0);
//        namespace.publishCommand("shoot", new Shoot(Shooter.getInstance(), () -> 0.05));
//        namespace.publishCommand("smart shoot", new SmartShoot(Shooter.getInstance(), setPoint));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
//        namespace.update();
//        SmartShoot.updateNamespace();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        vision.setThrottle(true);
    }

    @Override
    public void disabledExit() {
        vision.setThrottle(false);
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
