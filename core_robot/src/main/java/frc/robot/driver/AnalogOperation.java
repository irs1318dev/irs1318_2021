package frc.robot.driver;

import frc.robot.mechanisms.ControlPanelMechanism;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainMoveRight,
    DriveTrainTurnAngleReference,
    DriveTrainTurnAngleGoal,
    DriveTrainTurnSpeed,
    DriveTrainRotationA,
    DriveTrainRotationB,
    DriveTrainPathXGoal,
    DriveTrainPathYGoal,
    DriveTrainPathXVelocityGoal,
    DriveTrainPathYVelocityGoal,
    DriveTrainPathAngleVelocityGoal,

    // Powercell Operations
    PowerCellFlywheelVelocity,
    PowerCellCarousel,

    // Climber operations
    ClimberWinch,

    // Control Panel operations
    ControlPanelSpinSpeed;
}
