// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;

public class Climber extends SubsystemBase {
    TalonFX climbMotor = new TalonFX(CAN.CLIMB_MOTOR);
    
    public Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climbMotor.getConfigurator().apply(config);        
    }

    public void setPower(double speed) {
        climbMotor.set(speed);
    }

    public void stop()
    {
        setPower(0);
    }
    @Override
    public void periodic() {
    }
}
