package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

public class Collect extends Command {
    
    Collector collector;
    Indexer indexer;
    DoubleSupplier pow;

    public Collect(DoubleSupplier pow, Collector collector, Indexer indexer) {
        this.pow = pow;
        this.collector = collector;
        this.indexer = indexer;

        addRequirements(collector, indexer);
    }

    @Override
    public void execute() {
        collector.setPower(pow.getAsDouble());
        indexer.setPower(pow.getAsDouble() * 0.3);
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
