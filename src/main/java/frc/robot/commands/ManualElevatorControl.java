// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class ManualElevatorControl extends CommandBase {

    Elevator elevator;
    DoubleSupplier up;
    DoubleSupplier down;

    public ManualElevatorControl(Elevator elevator, DoubleSupplier up, DoubleSupplier down) {
        this.elevator = elevator;
        this.up = up;
        this.down = down;

        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setPower(0d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (up.getAsDouble() > 0.1) {
            elevator.setPower(up.getAsDouble());
        } else if (down.getAsDouble() > 0.1) {
            elevator.setPower(-down.getAsDouble());
        } else {
            elevator.setPower(0d);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0d);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
