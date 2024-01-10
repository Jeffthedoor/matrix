package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class Collect extends CommandBase {
    
    Collector collector;
    DoubleSupplier pow;

    public Collect(DoubleSupplier pow, Collector collector) {
        this.pow = pow;

        addRequirements(collector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        collector.setPower(0d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        collector.setPower(pow.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        collector.setPower(0d);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
