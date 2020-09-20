package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", true),
    RobotTime("r.time", true),
    RobotMatch("r.match"),
    DriverIsAuto("driver.isAuto"),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    OffboardVisionX("rpi.x", true),
    OffboardVisionY("rpi.y", true),
    OffboardVisionDistance("rpi.distance", true),
    OffboardVisionHorizontalAngle("rpi.horizontalAngle", true),
    OffboardVisionEnableVision("rpi.enableVision", true),
    OffboardVisionEnableStream("rpi.enableStream", true),
    OffboardVisionEnableProcessing("rpi.enableProcessing", true),
    PositionNavxConnected("pos.navx_connected", true),
    PositionNavxAngle("pos.navx_angle", true),
    PositionNavxX("pos.navx_x"),
    PositionNavxY("pos.navx_y"),
    PositionNavxZ("pos.navx_z"),
    PositionStartingAngle("pos.startingAngle"),
    DriveTrainDriveVelocity("dt.drive_vel"),
    DriveTrainDrivePosition("dt.drive_pos"),
    DriveTrainDriveError("dt.drive_err"),
    DriveTrainDriveVelocityGoal("dt.drive_goal"),
    DriveTrainAngleVelocity("dt.angle_vel"),
    DriveTrainAnglePosition("dt.angle_pos"),
    DriveTrainAngleError("dt.angle_err"),
    DriveTrainAnglePositionGoal("dt.angle_goal");
    

    public final String value;
    public final boolean shouldLog;
    private LoggingKey(String value)
    {
        this(value, false);
    }

    private LoggingKey(String value, boolean shouldLog)
    {
        this.value = value;
        this.shouldLog = shouldLog;
    }
}
