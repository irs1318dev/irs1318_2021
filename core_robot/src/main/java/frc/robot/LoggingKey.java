package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", true),
    RobotTime("r.time", true),
    RobotMatch("r.match"),
    RobotCrash("r.crash", true),
    DriverMode("driver.mode"),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    AutonomousDSMessage("auto.dsMessage"),
    OffboardVisionX("rpi.x"),
    OffboardVisionY("rpi.y"),
    OffboardVisionWidth("rpi.width"),
    OffboardVisionHeight("rpi.height"),
    OffboardVisionAngle("rpi.angle"),
    OffboardVisionDistance("rpi.distance"),
    OffboardVisionHorizontalAngle("rpi.horizontalAngle"),
    OffboardVisionEnableVision("rpi.enableVision"),
    OffboardVisionEnableStream("rpi.enableStream"),
    OffboardVisionEnableProcessing("rpi.enableProcessing"), // why do some have True?
    OffboardVisionMissedHeartbeats("rpi.missedHeartbeats"),
    NavxConnected("navx.connected", true),
    NavxAngle("navx.angle"),
    NavxPitch("navx.pitch"),
    NavxRoll("navx.roll"),
    NavxYaw("navx.yaw"),
    NavxX("navx.x"),
    NavxY("navx.y"),
    NavxZ("navx.z"),
    NavxStartingAngle("navx.startingAngle"),
    PigeonState("pigeon.state", true),
    PigeonYaw("pigeon.yaw", true),
    PigeonPitch("pigeon.pitch"),
    PigeonRoll("pigeon.roll"),
    PigeonStartingYaw("pigeon.startingYaw"),

    DriveTrainDesiredAngle("dt.angle_goal"),
    DriveTrainAngle("dt.angle"),
    DriveTrainXPosition("dt.xpos"),
    DriveTrainYPosition("dt.ypos"),
    DriveTrainFieldOriented("dt.field"),
    DriveTrainMaintainPosition("dt.maintain_pos"),

    DriveTrainAbsoluteEncoderAngle1("dt.absenc_ang1", true),
    DriveTrainDriveVelocity1("dt.drive_vel1"),
    DriveTrainDrivePosition1("dt.drive_pos1"),
    DriveTrainDriveError1("dt.drive_err1"),
    DriveTrainDriveVelocityGoal1("dt.drive_goal1"),
    DriveTrainSteerVelocity1("dt.steer_vel1"),
    DriveTrainSteerPosition1("dt.steer_pos1"),
    DriveTrainSteerAngle1("dt.steer_ang1"),
    DriveTrainSteerError1("dt.steer_err1"),
    DriveTrainSteerPositionGoal1("dt.steer_goal1"),

    DriveTrainAbsoluteEncoderAngle2("dt.absenc_ang2", true),
    DriveTrainDriveVelocity2("dt.drive_vel2"),
    DriveTrainDrivePosition2("dt.drive_pos2"),
    DriveTrainDriveError2("dt.drive_err2"),
    DriveTrainDriveVelocityGoal2("dt.drive_goal2"),
    DriveTrainSteerVelocity2("dt.steer_vel2"),
    DriveTrainSteerPosition2("dt.steer_pos2"),
    DriveTrainSteerAngle2("dt.steer_ang2"),
    DriveTrainSteerError2("dt.steer_err2"),
    DriveTrainSteerPositionGoal2("dt.steer_goal2"),

    DriveTrainAbsoluteEncoderAngle3("dt.absenc_ang3", true),
    DriveTrainDriveVelocity3("dt.drive_vel3"),
    DriveTrainDrivePosition3("dt.drive_pos3"),
    DriveTrainDriveError3("dt.drive_err3"),
    DriveTrainDriveVelocityGoal3("dt.drive_goal3"),
    DriveTrainSteerVelocity3("dt.steer_vel3"),
    DriveTrainSteerPosition3("dt.steer_pos3"),
    DriveTrainSteerAngle3("dt.steer_ang3"),
    DriveTrainSteerError3("dt.steer_err3"),
    DriveTrainSteerPositionGoal3("dt.steer_goal3"),

    DriveTrainAbsoluteEncoderAngle4("dt.absenc_ang4", true),
    DriveTrainDriveVelocity4("dt.drive_vel4"),
    DriveTrainDrivePosition4("dt.drive_pos4"),
    DriveTrainDriveError4("dt.drive_err4"),
    DriveTrainDriveVelocityGoal4("dt.drive_goal4"),
    DriveTrainSteerVelocity4("dt.steer_vel4"),
    DriveTrainSteerPosition4("dt.steer_pos4"),
    DriveTrainSteerAngle4("dt.steer_ang4"),
    DriveTrainSteerError4("dt.steer_err4"),
    DriveTrainSteerPositionGoal4("dt.steer_goal4"),

    PowerCellIsIntaking("pc.intaking"),
    PowerCellIntakeExtended("pc.intake_extended"),
    PowerCellFlywheelVelocity("pc.flywheel_vel"),
    PowerCellFlywheelPosition("pc.flywheel_pos"),
    PowerCellFlywheelError("pc.flywheel_err"),
    PowerCellCarouselPower("pc.carousel_power"),
    PowerCellDesiredCarouselVelocity("pc.carousel_des_vel"),
    PowerCellFlywheelVelocitySetpoint("pc.flywheel_vel_sp"),

    PowerCellCarouselVelocity("pc.carousel_vel"),
    PowerCellCarouselPosition("pc.carousel_pos"),

    ControlPanelExtend("cp.extend");

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
