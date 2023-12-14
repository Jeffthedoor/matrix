package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.lib.pathplanner.com.pathplanner.lib.auto.PIDConstants;

/**
 * Class to hold all of the constants for the robot
 */
public final class Constants {

    // Spark max voltage compensation
    public static final double VOLTAGE_COMPENSATION = 12d;

    // Path to the blackout directory
    public static final Path BLACKOUT_PATH = Paths.get("home/lvuser/blackout");

    // Check if we're on blackout
    public static final boolean isBlackout() {
        return BLACKOUT_PATH.toFile().exists();
    }

    // Check if we're on gridlock
    public static final boolean isGridlock() {
        return !isBlackout();
    }

    public static final double COMP_LOG_PERIOD = .33;

    // Constants for xbox controlers
    public static final class ControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;
        public static final int BUTTON_PAD_CONTROLLER_PORT = 2;

        // Deadband, min, and max power for the controllers
        public static final double DEADBAND = 0.1d;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;
    }

    // Constants for our system tests    
    public static final class SystemTestConstants {
        // Drive Test Variables
        public static final int DEGREES_INTERVAL_INCREASE = 30;
        public static final int ANGLE_DEAD_ZONE = 3;
        public static final int MAX_ROTATIONS_PER_DIRECTION = 2;
    }

    // Constants for our drivetrain
    public static final class DrivetrainConstants {
        // Our drivetrain track width and Wheelbase
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.8125d);
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
            public static final double kP = 0.2;// .116d;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kF = 0.55;// 229d;
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
            // Gridlocks swerve module absolute encoder offsets
            public static final class Gridlock {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(193.535);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(145.547);
                // public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(198.721);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(199.688);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(210.938);
            }

            // Blackouts swerve module absolute encoder offsets
            public static final class Blackout {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(253.916);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(222.451);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(19.688);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(63.018);
            }
        }
    }

    // Constants for our elevator
    public static final class ElevatorConstants {
        // Motor configuration constants
        public static final boolean MOTOR_INVERT = false;
        public static final int CURRENT_LIMIT = 40;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        // PID gains for our elevator
        public static final double kP = .35d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kF = 0.007d;

        public static final double TOLERANCE = 1d;

        // Conversion factor for our elevator
        public static final double GEAR_RATIO = 16d / 1d; // Motor gear reduction / output shaft gear reduction
        public static final double SPROCKET_DIAMETER = 1.440d;
        public static final double POSITION_CONVERSION_FACTOR = 1 / GEAR_RATIO * SPROCKET_DIAMETER * Math.PI;

        // Min/max height in inches
        public static final double MAX_EXTENSION = 23.287d;
        public static final double MIN_EXTENSION = 0d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 1d;

        public static final double LOG_PERIOD = 0.19;

        // Elevator limit switch types
        public static final SparkMaxLimitSwitch.Type TOP_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type BOTTOM_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
    }


    // RobotMap Constants
    public static final class RobotMap {
        // CAN IDs
        public static final class CAN {
            // Power distrobution hub ID
            public static final int PDH = 21;

            // Front left CanIDs
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 2;
            public static final int FRONT_LEFT_CANCODER = 0;
            // Front right CanIDs
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_RIGHT_CANCODER = 1;
            // Back right CanIDs
            public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int BACK_RIGHT_CANCODER = 2;
            // Back left CanIDs
            public static final int BACK_LEFT_DRIVE_MOTOR = 7;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 8;
            public static final int BACK_LEFT_CANCODER = 3;
        }

        public static final class PWM {
            public static final int SERVO = 0;
        }

        public static final class i2c { //Lowercase to avoid conflict with wpilib's I2C class
            public static final I2C.Port COLOR_SENSOR = I2C.Port.kMXP;
        }
    }

    // Constants used for auto balancing
    public static final class AutoBalanceConstants {
        // Magnitude for being balance
        public static final double BALANCED_MAGNITUDE = 2.5;

        // Upper and lower magnitude thresholds for checking if we are on the charge station at all
        public static final double UPPER_MAGNITUDE_THRESHOLD = 11;
        public static final double LOWER_MAGNITUDE_THRESHOLD = 7;
        // Min and max speeds for our auto balance
        public static final double MIN_SPEED_THRESHOLD = 0.35;
        public static final double MAX_SPEED_THRESHOLD = 1.5;

        // Delay time for our auto balance after falling
        public static final double DELAY_TIME = 1.5;

        // Target X position for the middle of the charge station
        public static final double TARGET_X = 3.85;// 3.93;

        //log period for autobalance
        public static final double LOG_PERIOD = 0.2;

        // Gains for our auto balance
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;
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
        public static final double SERVO_UP = -1d;
        public static final double SERVO_DOWN = 0.2d;

    }

        public static enum SlotPosition { // Position of each colum of scoring nodes for AutoScoring
            slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8, slot9
        }

        public static final double MAX_ACCELERATION_MUL = 1;

        public static final double CONTROL_LENGTHS = 0.001;

        // Tolerance for auto align
        public static final double X_TOLERANCE = 7d;
        public static final double R_TOLERANCE = 5d;

        //Log period auto align
        public static final double LOG_PERIOD = 0.25;
    }
