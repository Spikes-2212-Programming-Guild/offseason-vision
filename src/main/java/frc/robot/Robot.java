// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConstants;
import frc.robot.vision.Vision;

public class Robot extends TimedRobot {

    private final Swerve swerve;
    private final Vision vision;

    public Robot() {
        swerve = Swerve.getInstance();
        vision = Vision.getInstance();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SwerveConstants.updateNamespace();
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
        OI oi = new OI();
        swerve.resetRelativeEncoders();
        swerve.resetFieldRelativity();
        swerve.setDefaultCommand(swerve.driveWithController(
                oi::getLeftX,
                oi::getLeftY,
                oi::getRightX, true, 1
        ));
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
