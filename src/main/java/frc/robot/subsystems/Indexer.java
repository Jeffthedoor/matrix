// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;

public class Indexer extends SubsystemBase {

    CANSparkMax indexerMotor;

    public Indexer() {
        indexerMotor = new CANSparkMax(CAN.INDEXER_MOTOR, MotorType.kBrushless);
    }

    public void setPower(double pow) {
        indexerMotor.set(pow);
    }

    public void stop() {
        indexerMotor.set(0d);
    }
}
