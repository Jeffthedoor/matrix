package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.Collect;
import frc.robot.lib.LightningContainer;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveModule;

public class RobotContainer extends LightningContainer {
    private Drivetrain drivetrain;
    private Collector collector;

    private static XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    // private SendableChooser<Command> autoChooser;

    @Override
	protected void initializeSubsystems() {
        drivetrain = new Drivetrain();
        collector = new Collector();

		// autoChooser = AutoBuilder.buildAutoChooser();
		// LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    protected void configureButtonBindings() {
        new Trigger(driverController::getStartButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(drivetrain::lock, drivetrain));
    }

    @Override
    protected Command getAutonomousCommands() {
        // return autoChooser.getSelected();
        return null;
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            // new AbsoluteFieldDrive(
            //     drivetrain,
            //     () -> MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.DEADBAND),
            //     () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.DEADBAND),
            //     () -> -driverController.getRightX(),
            //     () -> -driverController.getRightY()));
            new AbsoluteFieldDrive(drivetrain,
                () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getLeftY(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.DEADBAND)));

        collector.setDefaultCommand(new Collect(collector, ()-> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())));

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
}
