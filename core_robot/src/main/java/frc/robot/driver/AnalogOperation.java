package frc.robot.driver;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainMoveSide,
    DriveTrainTurnX,
    DriveTrainTurnY,
    DriveTrainRotationA,
    DriveTrainRotationB,
    DriveTrainPathXGoal,
    DriveTrainPathYGoal,
    DriveTrainPathAngleGoal,
    DriveTrainVelocityGoal;
}
