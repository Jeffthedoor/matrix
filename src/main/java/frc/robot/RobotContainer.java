package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.Climb;
import frc.robot.lib.LightningContainer;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveModule;

public class RobotContainer extends LightningContainer {
    private Drivetrain drivetrain;
    // private Climber climber;

    private static XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    // private SendableChooser<Command> autoChooser;

    @Override
	protected void initializeSubsystems() {
        drivetrain = new Drivetrain();
        // climber = new Climber();

		// autoChooser = AutoBuilder.buildAutoChooser();
		// LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    protected void configureButtonBindings() {
        
        drivetrain.zeroGyro();
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
            new AbsoluteDrive(
                drivetrain,
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.DEADBAND),
                () -> -driverController.getRightX(),
                () -> -driverController.getRightY()));

        // climber.setDefaultCommand(new Climb(climber, ()-> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())));

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
