// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;

public class Indexer extends SubsystemBase {
    TalonFX motor = new TalonFX(CAN.INDEXER_MOTOR);
    
    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor.getConfigurator().apply(config);        
    }

    public void setPower(double speed) {
        motor.set(speed);
    }

    public void stop() 
{
    setPower(0);
}
    @Override
    public void periodic() {
    }
}
