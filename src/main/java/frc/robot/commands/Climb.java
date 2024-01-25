package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class Climb extends Command {
    
    Climber climber;
    DoubleSupplier pow;

    public Climb(DoubleSupplier pow, Climber climber) {
        this.pow = pow;
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setPower(pow.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
