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


    DriveTrainAbsoluteEncoderPosition1("dt.absenc_pos1"),
    DriveTrainDriveVelocity1("dt.drive_vel1"),
    DriveTrainDrivePosition1("dt.drive_pos1"),
    DriveTrainDriveError1("dt.drive_err1"),
    DriveTrainDriveVelocityGoal1("dt.drive_goal1"),
    DriveTrainAngleVelocity1("dt.angle_vel1"),
    DriveTrainAnglePosition1("dt.angle_pos1"),
    DriveTrainAngleError1("dt.angle_err1"),
    DriveTrainAnglePositionGoal1("dt.angle_goal1"),

    DriveTrainAbsoluteEncoderPosition2("dt.absenc_pos2"),
    DriveTrainDriveVelocity2("dt.drive_vel2"),
    DriveTrainDrivePosition2("dt.drive_pos2"),
    DriveTrainDriveError2("dt.drive_err2"),
    DriveTrainDriveVelocityGoal2("dt.drive_goal2"),
    DriveTrainAngleVelocity2("dt.angle_vel2"),
    DriveTrainAnglePosition2("dt.angle_pos2"),
    DriveTrainAngleError2("dt.angle_err2"),
    DriveTrainAnglePositionGoal2("dt.angle_goal2"),

    DriveTrainAbsoluteEncoderPosition3("dt.absenc_pos3"),
    DriveTrainDriveVelocity3("dt.drive_vel3"),
    DriveTrainDrivePosition3("dt.drive_pos3"),
    DriveTrainDriveError3("dt.drive_err3"),
    DriveTrainDriveVelocityGoal3("dt.drive_goal3"),
    DriveTrainAngleVelocity3("dt.angle_vel3"),
    DriveTrainAnglePosition3("dt.angle_pos3"),
    DriveTrainAngleError3("dt.angle_err3"),
    DriveTrainAnglePositionGoal3("dt.angle_goal3"),

    DriveTrainAbsoluteEncoderPosition4("dt.absenc_pos4"),
    DriveTrainDriveVelocity4("dt.drive_vel4"),
    DriveTrainDrivePosition4("dt.drive_pos4"),
    DriveTrainDriveError4("dt.drive_err4"),
    DriveTrainDriveVelocityGoal4("dt.drive_goal4"),
    DriveTrainAngleVelocity4("dt.angle_vel4"),
    DriveTrainAnglePosition4("dt.angle_pos4"),
    DriveTrainAngleError4("dt.angle_err4"),
    DriveTrainAnglePositionGoal4("dt.angle_goal4");

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
