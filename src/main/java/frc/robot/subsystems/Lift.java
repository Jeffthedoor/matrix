package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftStates;
import frc.robot.Constants.XYConstants;

public class Lift extends SubsystemBase {

    public enum States {
        groundCollect,
        doubleSubstationCollect,
        lowScore,
        mediumScore,
        highScore,
        stowed
    }

    Elevator elevator;
    Wrist wrist;
    Arm arm;

    public States state = States.stowed; 
    private Translation2d position = new Translation2d();

    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator; 
        this.wrist = wrist;
        this.arm = arm;
    }

    public void setState(States state){
        this.state = state;
    }

    public Translation2d getElevatorXY() {
        return new Translation2d(elevator.getHeight(), XYConstants.ELEVATOR_ANGLE);
    }

    public Translation2d getBarXY() {
        return new Translation2d(XYConstants.ARM_RADIUS, new Rotation2d(Math.toRadians(arm.getAngle())));
    }

    public Translation2d getOverallXY() {
        return XYConstants.ELEVATOR_OFFSET.plus(getElevatorXY()).plus(getBarXY().plus(XYConstants.COLLECTOR_OFFSET));
    }

    public Boolean isReachable(Translation2d pose) {
        return XYConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }



    @Override
    public void periodic() {
        switch(state) {
            case groundCollect:
                position = LiftStates.GROUND_COLLECT;
            break;

            case doubleSubstationCollect:
                position = LiftStates.DOUBLE_SUBSTATION_COLLECT;
            break;

            case lowScore:
                position = LiftStates.LOW_SCORE;
            break;

            case mediumScore:
                position = LiftStates.MEDIUM_SCORE;
            break;

            case highScore:
                position = LiftStates.HIGH_SCORE;
            break;

            case stowed:
                position = LiftStates.STOWED;
            break;
        }

        if(isReachable(position)) {
            Translation2d currentPose = getOverallXY();
            Translation2d desiredPose = position;

            Translation2d delta = desiredPose.minus(currentPose);

            Rotation2d armAngle = delta.getAngle().minus(XYConstants.ELEVATOR_ANGLE);
            double elevatorHeight = delta.getNorm() * Math.cos(armAngle.getDegrees());

            elevator.setHeight(elevatorHeight);
            arm.setAngle(armAngle.getDegrees());
            wrist.setAngle(armAngle.getDegrees() + 90); //math
        }
    }
}
