package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class AutonShoot extends Command {
    Shooter shooter;
    Collector collector;
    Indexer indexer;
    public AutonShoot(Shooter shooter, Collector collector, Indexer indexer) {
        this.shooter = shooter;
        this.collector = collector;
        this.indexer = indexer;
        addRequirements(shooter, collector, indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
