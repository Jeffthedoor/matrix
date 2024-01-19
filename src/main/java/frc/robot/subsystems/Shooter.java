// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.lib.shuffleboard.LightningShuffleboard;

public class Shooter extends SubsystemBase {
    CANSparkMax motor1;
    CANSparkMax motor2;

    public Shooter() {
        motor1 = new CANSparkMax(CAN.SHOOTER_MOTOR1, MotorType.kBrushless);
        motor2 = new CANSparkMax(CAN.SHOOTER_MOTOR2, MotorType.kBrushless);

        motor1.setInverted(false);
        motor2.setInverted(true);
        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
    }

    public double getRPM() {
        return (motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity()) / 2;
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("shooter", "RPM", getRPM());
    }

    public void setPower(double pow) {
        motor1.set(pow);
        motor2.set(pow);
    }
}
