// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;

public class Collector extends SubsystemBase {

    CANSparkMax motor;

    public Collector() {
        motor = new CANSparkMax(CAN.COLLECTOR_MOTOR, MotorType.kBrushless);

        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void setPower(double pow) {
        motor.set(pow);
    }

    public void stop() {
        motor.set(0d);
    }
}
