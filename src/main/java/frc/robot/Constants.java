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
    public static final boolean isHurleyBot() {
        return !isHurley();
    }

    // Constants for xbox controlers
    public static final class ControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;

        // Deadband, min, and max power for the controllers
        public static final double DEADBAND = 0.1d;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;
    }

    // Constants for our drivetrain
    public static final class DrivetrainConstants {
        // Our drivetrain track width and Wheelbase
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.8125d); //TODO FIND and get for each bot
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.8125d);

        // Module resting/default angles
        public static final double FRONT_LEFT_RESTING_ANGLE = Math.toRadians(-45d);
        public static final double FRONT_RIGHT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_LEFT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_RIGHT_RESTING_ANGLE = Math.toRadians(-45d);

        // Our max voltage, velocity, angular velocity, and angular acceleration
        public static final double MAX_VOLTAGE = 12;
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = +MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 2 * Math.PI / 5;

        // Module configuration constants
        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int STEER_CURRENT_LIMIT = 30;
        public static final double NOMINAL_VOLTAGE = 12d;

        public static final double LOG_PERIOD = 0.18;

        public static final double SLOW_MODE_TRANSLATIONAL_MULT = 0.7;
        public static final double SLOW_MODE_ROTATIONAL_MULT = 0.5;

        // Pigeon heading offset
        public static final Rotation2d HEADING_OFFSET = Rotation2d.fromDegrees(90);

        // Standard dev for robot pose
        public static final Matrix<N3, N1> STANDARD_DEV_POSE_MATRIX = VecBuilder.fill(0.1, 0.1, 0.1);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.2;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kF = 0.55;
        }

        // Gains vaules for theta PIDControllers
        public static final class ThetaGains {
            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;
        }

        // PID gains for our heading compensation
        public static final class HeadingGains {
            public static final double kP = 0.005d;
            public static final double kI = 0d;
            public static final double kD = 0d;
        }

        // Steer offsets for our modules
        public static final class Offsets {
            // Hurley bot swerve module absolute encoder offsets
            public static final class Hurley {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(279.242); //TDO refind
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(358.661);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(357.321);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(10.045);
            }

            // Howitzer swerve module absolute encoder offsets
            public static final class Howitzer {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(347.695);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(1.406);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(12.392);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(58.886);
            }
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

            public static final int COLLECTOR_MOTOR = 13;
        }
    }

    // Constants for autonomous
    public static final class AutonomousConstants {
        // Path planner PIDConstants
        public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(2.5, 0, 0); // Drive velocity PID 10.5
        public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(4, 0, 0); // Rotation PID 7
        public static final PIDConstants POSE_PID_CONSTANTS = new PIDConstants(0, 0, 0); // X and Y position PID

        // Max velocity and acceleration for the path planner
        public static final double MAX_VELOCITY = 2;
        public static final double MAX_ACCELERATION = 1;
    }
}
