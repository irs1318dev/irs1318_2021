package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDBrake,

    // Testing operations
    FollowSomePath,
    FollowAnotherPath,
    FollowADifferentPath,
}
