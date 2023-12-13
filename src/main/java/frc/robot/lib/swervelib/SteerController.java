package frc.robot.lib.swervelib;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void setMotorEncoderAngle();

    double getTemperature();
}

