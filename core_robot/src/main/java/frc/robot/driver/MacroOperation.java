package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDBrake,

    // Testing operations
    VisionCenter,
    VisionCenterAndAdvance,
    FollowSomePath,
    FollowAnotherPath,
    FollowADifferentPath,
}
