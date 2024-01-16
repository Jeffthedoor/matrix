package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.pathplanner.com.pathplanner.lib.auto.PIDConstants;
import swervelib.parser.PIDFConfig;

/**
 * Class to hold all of the constants for the robot
 */
public final class Constants {

    // Spark max voltage compensation
    public static final double VOLTAGE_COMPENSATION = 12d;

    // Path to the blackout directory
    public static final Path HURLEY_PATH = Paths.get("home/lvuser/Hurley");

    // Check if we're on blackout
    public static final boolean isHurley() {
        return HURLEY_PATH.toFile().exists();
    }

    // Check if we're on gridlock
    public static final boolean isHowitzer() {
        return !isHurley();
    }

    // Constants for xbox controlers
    public static final class ControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;

        // Deadband, min, and max power for the controllers
        public static final double DEADBAND = 0.1d;
        public static final double FLICKSTICK_DEADBAND = 0.15d;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;
    }

    // Constants for our drivetrain
    public static final class DrivetrainConstants {
        public static final double AZIMUTH_RATIO = 14.2587d;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        public static final double DRIVE_RATIO = 6.8571;

        public static final double MAX_SPEED = Units.feetToMeters(14.4);
        // Hurley offsets
        // public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(279.242); //TODO refind
        // public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(358.661);
        // public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(357.321);
        // public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(10.045);
        public static final double FRONT_LEFT_STEER_OFFSET = 244.77;
        public static final double FRONT_RIGHT_STEER_OFFSET = 335.923;
        public static final double BACK_LEFT_STEER_OFFSET = 31.632;
        public static final double BACK_RIGHT_STEER_OFFSET = 211.53;


        public static final class AutonConstants {
            //TODO: copy pasted values, need to be changed
            public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
            public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
            public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

            public static final double AUTO_MAX_SPEED = DrivetrainConstants.MAX_SPEED;
            public static final double MAX_ACCELERATION = 2d;
        }
    }

    // RobotMap Constants
    public static final class RobotMap {
        // CAN IDs
        public static final class CAN {
            // Power distrobution hub ID
            public static final int PDH = 21;

            // Front left CanIDs
            public static final int FRONT_LEFT_DRIVE_MOTOR = 3;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_LEFT_CANCODER = 10;
            // Front right CanIDs
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 5;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int FRONT_RIGHT_CANCODER = 11;
            // Back right CanIDs
            public static final int BACK_RIGHT_DRIVE_MOTOR = 7;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 8;
            public static final int BACK_RIGHT_CANCODER = 12;
            // Back left CanIDs
            public static final int BACK_LEFT_DRIVE_MOTOR = 1;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 2;
            public static final int BACK_LEFT_CANCODER = 9;
        }
    }

    
}
