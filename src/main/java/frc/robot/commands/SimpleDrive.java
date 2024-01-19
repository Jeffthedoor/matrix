package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.lib.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SimpleDrive extends Command
{

  private final Drivetrain swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier vRot;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
   *                          looking through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
   *                          robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
   *                          no deadband.  Positive is towards the left wall when looking through the driver station
   *                          glass.
   * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
   *                          robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
   *                          with no deadband. Positive is away from the alliance wall.
   */
  public SimpleDrive(Drivetrain swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRot) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.vRot = vRot;



    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    swerve.setMotorBrake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    // Make the robot move
    swerve.drive(new Translation2d(
        vX.getAsDouble()*DrivetrainConstants.MAX_SPEED,
        vY.getAsDouble()*DrivetrainConstants.MAX_SPEED), 
        vRot.getAsDouble()*DrivetrainConstants.MAX_ROT_SPEED, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}