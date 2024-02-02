package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * Class to hold all of the constants for the robot
 */
public final class Constants {
    // Constants for xbox controlers
    public static final class ControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;

        
        // Deadband, for joysticks
        public static final double DEADBAND = 0.1d;
    }
    
    // Constants for our drivetrain
    public static final class DrivetrainConstants {
        public static final double SPEED_MULTIPLIER = 0.8;
        public static final double ROTATION_MULTIPLIER = 0.6;
        
        public static final Rotation3d GYRO_OFFSET = new Rotation3d(0, 0, Units.degreesToRadians(90));

        public static final double AZIMUTH_RATIO = 12.8;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        public static final double DRIVE_RATIO = 6.857142857142858;

        public static final double MAX_SPEED = Units.feetToMeters(14.4);

        //No idea how well this works :shrug:
        public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(Math.sqrt(2*Math.pow(13d, 2d)));

        public static final double MAX_ROT_SPEED = 6;


        public static final class AutonConstants {
            //TODO: copy pasted values, need to be changed
            public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
            public static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);

            public static final double AUTO_MAX_SPEED = DrivetrainConstants.MAX_SPEED;
            public static final double MAX_ACCELERATION = 2d;
        }
    }

    // RobotMap Constants
    public static final class RobotMap {
        // CAN IDs
        public static final class CAN {
            // Power distrobution hub ID
            public static final int PDH = 25;

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
            public static final int SHOOTER_MOTOR1 = 15; //right flywheel
            public static final int SHOOTER_MOTOR2 = 14;
            public static final int INDEXER_MOTOR = 16;
            public static final int CLIMB_MOTOR = 17;
        }
    }

    public static final double INDEX_POWER = 1;
    public static final double INDEX_DEBOUNCE_TIME = 0.1;
    
}
