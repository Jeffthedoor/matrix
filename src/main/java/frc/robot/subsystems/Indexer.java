// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.lib.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {
    TalonFX motor = new TalonFX(CAN.INDEXER_MOTOR);

    DigitalInput beamBreak = new DigitalInput(0);

    Debouncer debouncer = new Debouncer(Constants.INDEX_DEBOUNCE_TIME);

    boolean beamBreakState = false;
    
    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        LightningShuffleboard.setBoolSupplier("Indexer", "BeamBreak", () -> getBeamBreak());

        motor.getConfigurator().apply(config);        
    }

    public void setPower(double speed) {
        motor.set(speed);
    }

    public void stop() 
    {
        setPower(0);
    }

    public boolean getBeamBreak() {
        return debouncer.calculate(beamBreak.get());
    }

    @Override
    public void periodic() {
        
    }
}
