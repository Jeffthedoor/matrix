package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    Climber climber;
    DoubleSupplier speed;

    public Climb(Climber climber, DoubleSupplier speed) {
        this.climber = climber;
        this.speed = speed;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climber.climb(speed.getAsDouble() * 0.5);
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
