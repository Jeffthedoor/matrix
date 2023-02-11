package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.math.LightningMath;

public class Wrist extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController wristController;
    private SparkMaxAbsoluteEncoder encoder;
    private double OFFSET;
    private double targetAngle;

    public Wrist() {
        if (Constants.isBlackout()) {
            OFFSET = WristConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            OFFSET = WristConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, WristConstants.MOTOR_INVERT,
                WristConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMP_VOLTAGE,
                WristConstants.MOTOR_TYPE, WristConstants.NEUTRAL_MODE);
        wristController = NeoConfig.createPIDController(motor.getPIDController(),
                new SparkMaxPIDGains(WristConstants.kP, WristConstants.kI, WristConstants.kD,
                        WristConstants.kF));
        encoder = NeoConfig.createAbsoluteEncoder(motor, OFFSET);
        encoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * 
     * @return Rotation2d of the wrist from encoder
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }

    /**
     * Takes a rotation2d and sets the wrist to that angle bounded by the min and max angles
     * 
     * @param angle Rotation2d to set the wrist to
     */
    public void setAngle(Rotation2d angle) {
        targetAngle = MathUtil.clamp(angle.getDegrees(), WristConstants.MIN_ANGLE,
                WristConstants.MAX_ANGLE);
        wristController.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        wristController = NeoConfig.createPIDController(wristController,
                new SparkMaxPIDGains(kP, kI, kD, kF));
    }

    public void setPower(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0d);
    }

    public boolean onTarget() {
        return Math.abs(encoder.getPosition() - targetAngle) < WristConstants.TOLERANCE;
    }
}
