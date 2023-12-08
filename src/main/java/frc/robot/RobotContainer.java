package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.LightningContainer;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    // Creating our main subsystems
    private static final Drivetrain drivetrain = new Drivetrain();

    // Creates our controllers and deadzones
    private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    // creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        /* driver Controls */
        // RESETS
        new Trigger(() -> (driver.getBackButton() && driver.getStartButton())).onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain)); // Resets Forward to be the direction the robot is facing

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle)); // REsyncs the NEOs relative encoder to the absolute encoders on the swerve modules

        // Flips modules 180 degrees to fix a module that is facing the wrong way on startup
        new Trigger(() -> driver.getPOV() == 0).onTrue(new InstantCommand(drivetrain::flipFR, drivetrain));
        new Trigger(() -> driver.getPOV() == 180).onTrue(new InstantCommand(drivetrain::flipBL, drivetrain));
        new Trigger(() -> driver.getPOV() == 90).onTrue(new InstantCommand(drivetrain::flipBR, drivetrain));
        new Trigger(() -> driver.getPOV() == 270).onTrue(new InstantCommand(drivetrain::flipFL, drivetrain));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {}

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND),
        () -> driver.getRightTriggerAxis() > 0.25, () -> driver.getLeftTriggerAxis() > 0.25));

    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}


    /* Hello this is your favorite programing monster
     * 
     * I haunt your code and your dreams, you wont sleep at night while thinking about 
     * the code issues you've been having. 
     * 
     * have fun
     * 
     * bu bye >:)
     */




}
