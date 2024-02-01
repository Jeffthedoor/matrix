package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.AutonShoot;
import frc.robot.commands.Climb;
import frc.robot.commands.Collect;
import frc.robot.commands.Shoot;
import frc.robot.commands.SimpleDrive;
import frc.robot.lib.LightningContainer;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer extends LightningContainer {
    private Drivetrain drivetrain;
    private Collector collector;
    private Indexer indexer;
    private Shooter shooter;
    private Climber climber;

    private static XboxController driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private static XboxController copilotController = new XboxController(ControllerConstants.COPILOT_CONTROLLER_PORT);
    // private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    @Override
	protected void initializeSubsystems() {
        collector = new Collector();
        shooter = new Shooter();
        indexer = new Indexer();
        climber = new Climber();
		// LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    protected void configureButtonBindings() {
        
        this.drivetrain = new Drivetrain();
        new Trigger(driverController::getStartButton).onTrue(new InstantCommand(drivetrain::zeroGyro));
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(drivetrain::lock, drivetrain));

        new Trigger(copilotController::getAButton).toggleOnTrue(new Shoot(indexer, shooter));

    }

    @Override
    protected Command getAutonomousCommands() {
        // NamedCommands.registerCommand("startShooting", new AutonShoot(shooter, collector, indexer));
        // return autoChooser.getSelected();
        return null;
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            new SimpleDrive(
                drivetrain,
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), ControllerConstants.DEADBAND) * DrivetrainConstants.SPEED_MULTIPLIER,
                () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.DEADBAND) * DrivetrainConstants.SPEED_MULTIPLIER,
                () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.DEADBAND * DrivetrainConstants.ROTATION_MULTIPLIER)));

        collector.setDefaultCommand(new Collect(()-> (copilotController.getRightTriggerAxis() - copilotController.getLeftTriggerAxis()), collector, indexer));
        climber.setDefaultCommand(new Climb(() -> -MathUtil.applyDeadband(copilotController.getLeftY(), 0.1), climber));
        
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
