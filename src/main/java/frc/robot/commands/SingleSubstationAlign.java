package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Collector.GamePiece;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SingleSubstationAlign extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private PIDController Xcontroller = new PIDController(-0.05, 0, -0.03);
    private PIDController Rcontroller = new PIDController(0.005, 0, 0);
    private Collector collector;
    private double XOutput = 0;
    private double ROutput = 0;
    private double distance = 0;

    public SingleSubstationAlign(Drivetrain drivetrain, LimelightFront limelightFront, Collector collector) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;
        this.collector = collector;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(3); // TODO see if pipeline 10 is working for non game tags

        Xcontroller.setSetpoint(0d);
        Xcontroller.setTolerance(AutoAlignConstants.X_TOLERANCE);

        Rcontroller.setSetpoint(90);// TODO figure out what it is to face the wall Might be different between red and blue
        Rcontroller.setTolerance(AutoAlignConstants.R_TOLERANCE);
    }

    //90 is 180

    @Override
    public void execute() {
        if (limelightFront.hasVision() && limelightFront.getPipelineNum() == 3) {
            if (drivetrain.getYaw2d().getDegrees() - 0 < AutoAlignConstants.R_TOLERANCE) {
                distance = limelightFront.getHorizontalOffset();
                XOutput = Xcontroller.calculate(distance);
            } else {
                XOutput = 0d;
            }
        } else {
            XOutput = 0d;
        }
        
        if (drivetrain.getYaw2d().getDegrees() - 0 < AutoAlignConstants.R_TOLERANCE) {
            ROutput = 0;
        } else {
            ROutput = Rcontroller.calculate(drivetrain.getYaw2d().getDegrees());
        }


        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            drivetrain.percentOutputToMetersPerSecond(XOutput),
            drivetrain.percentOutputToMetersPerSecond(0), 
            drivetrain.percentOutputToRadiansPerSecond(ROutput), 
            drivetrain.getYaw2d()));

        
        LightningShuffleboard.setDouble("April-Align", "horizontal offset", distance);
        LightningShuffleboard.setDouble("April-Align", "X Pid output", Xcontroller.calculate(distance));
        LightningShuffleboard.setDouble("April-Align", "R Pid output", Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()));
        LightningShuffleboard.setDouble("April-Align", "ROutput", ROutput);
        LightningShuffleboard.setDouble("April-Align", "XOutput", XOutput);

        Xcontroller.setP(LightningShuffleboard.getDouble("April-Align", "X Pee", Xcontroller.getP()));
        Xcontroller.setD(LightningShuffleboard.getDouble("April-Align", "X D", Xcontroller.getD()));

        Rcontroller.setP(LightningShuffleboard.getDouble("April-Align", "R Pee", Rcontroller.getP()));
        Rcontroller.setD(LightningShuffleboard.getDouble("April-Align", "R D", Rcontroller.getD()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        limelightFront.setPipelineNum(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
