package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.AutonConstants;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.lib.shuffleboard.LightningShuffleboardPeriodic;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase {
	private final SwerveDrive swerveDrive;

	public final double maxSpeed = DrivetrainConstants.MAX_SPEED;

	public Drivetrain() {

		double angleConversionFactor = SwerveMath
				.calculateDegreesPerSteeringRotation(DrivetrainConstants.AZIMUTH_RATIO, 1);

		double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
				DrivetrainConstants.WHEEL_DIAMETER, DrivetrainConstants.DRIVE_RATIO, 1);
		try {
			swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
					.createSwerveDrive(maxSpeed, angleConversionFactor, driveConversionFactor);
		} catch (IOException e) {
			System.out.println(e);
			throw new RuntimeException(e);
		}

		swerveDrive.setGyroOffset(DrivetrainConstants.GYRO_OFFSET);
		swerveDrive.zeroGyro();


		CommandScheduler.getInstance().registerSubsystem(this);


	}

	public void configureAuton() {
		// TODO test if these are all the right supplier
		AutoBuilder.configureHolonomic(
			this::getPose, // Robot pose supplier
			this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
			this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
			new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
				AutonConstants.TRANSLATION_PID, // Translation PID constants
				AutonConstants.ROTATION_PID, // Rotation PID constants
				DrivetrainConstants.MAX_SPEED, // Max module speed, in m/s
				DrivetrainConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
				new ReplanningConfig() // Default path replanning config. See the API for the options here
				), () -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				}, this // Reference to this subsystem to set requirements
		);
	}

	/**
	 * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a
	 * rotation rate, and calculates and commands module states accordingly. Can use either
	 * open-loop or closed-loop velocity control for the wheel velocities. Also has field- and
	 * robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation {@link Translation2d} that is the commanded linear velocity of the robot,
	 *        in meters per second. In robot-relative mode, positive x is torwards the bow (front)
	 *        and positive y is torwards port (left). In field-relative mode, positive x is away
	 *        from the alliance wall (field North) and positive y is torwards the left wall when
	 *        looking through the driver station glass (field West).
	 * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
	 *        field/robot relativity.
	 * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}


	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveDriveKinematics} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset
	 * when calling this method. However, if either gyro angle or module position is reset, this
	 * must be called in order for odometry to keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
		swerveDrive.setGyroOffset(new Rotation3d(0, 0, 0));

	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public SwerveModule[] getSwerveModules() {
		return swerveDrive.getModules();
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading() {
		return swerveDrive.getYaw();
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
	 * direction. The other for the angle of the robot.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param headingX X joystick which controls the angle of the robot.
	 * @param headingY Y joystick which controls the angle of the robot.
	 * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX,
			double headingY) {
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
				getHeading().getRadians(), maxSpeed);
	}

	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param angle The angle in as a {@link Rotation2d}.
	 * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(),
				getHeading().getRadians(), maxSpeed);
	}

	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot
	 *
	 * @return A ChassisSpeeds object of the current field-relative velocity
	 */
	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} fpr the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock() {
		swerveDrive.lockPose();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	/**
	 * Add a fake vision reading for testing purposes.
	 */
	public void addFakeVisionReading() {
		swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)),
				Timer.getFPGATimestamp());
	}

	/**
	 * Factory to fetch the PathPlanner command to follow the defined path.
	 *
	 * @param path Path planner path to specify.
	 * @param constraints {@link PathConstraints} for
	 *        {@link com.pathplanner.lib.PathPlanner#loadPathGroup} function limiting velocity and
	 *        acceleration.
	 * @param eventMap {@link java.util.HashMap} of commands corresponding to path planner events
	 *        given as strings.
	 * @param translation The {@link PIDConstants} for the translation of the robot while following
	 *        the path.
	 * @param rotation The {@link PIDConstants} for the rotation of the robot while following the
	 *        path.
	 * @param useAllianceColor Automatically transform the path based on alliance color.
	 * @return PathPlanner command to follow the given path.
	 */
	// public Command creatPathPlannerCommand(String path, PathConstraints constraints, Map<String,
	// Command> eventMap,
	// PIDConstants translation, PIDConstants rotation, PIDConstants pose, boolean useAllianceColor)
	// {

	@Override
	public void periodic() {
		SwerveModule[] states = getSwerveModules();
		LightningShuffleboard.setDouble("Drivetrain", "FrontRightAngle",
				states[1].getAbsolutePosition());
		LightningShuffleboard.setDouble("Drivetrain", "FrontLeftAngle",
				states[0].getAbsolutePosition());
		LightningShuffleboard.setDouble("Drivetrain", "BackRightAngle",
				states[3].getAbsolutePosition());
		LightningShuffleboard.setDouble("Drivetrain", "BackLeftAngle",
				states[2].getAbsolutePosition());

		LightningShuffleboard.setDouble("Drivetrain", "Odo Roll",
				swerveDrive.getRoll().getDegrees());
		LightningShuffleboard.setDouble("Drivetrain", "Odo Yaw", swerveDrive.getYaw().getDegrees());

	}
}
