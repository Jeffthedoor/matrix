package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.lib.LightningContainer;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveModule;

public class RobotContainer extends LightningContainer {
    private Drivetrain drivetrain;

    private static XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    @Override
    protected void configureButtonBindings() {
        this.drivetrain = new Drivetrain();
        drivetrain.zeroGyro();
        new Trigger(driverController::getStartButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(drivetrain::lock, drivetrain));
    }

    @Override
    protected void configureAutonomousCommands() {}

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            new AbsoluteDrive(
                drivetrain,
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getRightX(), ControllerConstants.FLICKSTICK_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getRightY(), ControllerConstants.FLICKSTICK_DEADBAND)));
    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {
    }

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}
}
