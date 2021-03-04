package frc.robot.driver;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainMoveSide,
    DriveTrainTurnAngleReference,
    DriveTrainTurnAngleGoal,
    DriveTrainTurnSpeed,
    DriveTrainRotationA,
    DriveTrainRotationB,
    DriveTrainPathXGoal,
    DriveTrainPathYGoal,
    DriveTrainPathVelocityGoal;
}
