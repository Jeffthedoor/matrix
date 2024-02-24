package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Collect extends Command {
    Collector climber;
    DoubleSupplier speed;

    public Collect(Collector climber, DoubleSupplier speed) {
        this.climber = climber;
        this.speed = speed;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climber.climb(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climber.climb(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
