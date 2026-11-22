package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class Shoot extends Command {
    private final Shooter shooter;
    private final Supplier<Double> speed;

    public Shoot(Shooter shooter, Supplier<Double> speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void execute() {
        shooter.setSpeed(speed.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.finish();
    }
}
