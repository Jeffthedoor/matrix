package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants.AutonConstants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends SequentialCommandGroup
{

  public FollowTrajectory(Drivetrain drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry)
  {
    addRequirements(drivebase);

    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            AutonConstants.xAutoPID.createPIDController(),
            AutonConstants.yAutoPID.createPIDController(),
            AutonConstants.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            drivebase)
               );
  }
}