package frc.robot;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    //================================================= Vision ======================================================

    // Vision Alignment 
    public static final double CAMERA_PITCH = 22.5; // in degrees
    public static final double CAMERA_X_OFFSET = 0.0; // in inches
    public static final double CAMERA_Z_OFFSET = 18.0; // in inches
    public static final double VISIONTARGET_Z_OFFSET = 90.25; // in inches
    public static final double CAMERA_TO_TARGET_Z_OFFSET = HardwareConstants.VISIONTARGET_Z_OFFSET - HardwareConstants.CAMERA_Z_OFFSET;
    public static final double CAMERA_YAW = 0.0; // in degrees

    //================================================== DriveTrain ==============================================================

    public static final double DRIVETRAIN_ENCODER_DEGREES_PER_VOLT = -72.0;

    public static final boolean DRIVETRAIN_STEER_MOTOR1_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR1_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR2_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR2_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR3_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR3_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR4_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_STEER_MOTOR4_INVERT_SENSOR = false;

    public static final boolean DRIVETRAIN_DRIVE_MOTOR1_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR1_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR2_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR2_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR3_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR3_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR4_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_DRIVE_MOTOR4_INVERT_SENSOR = false;

    public static final double DRIVETRAIN_STEER_ENCODER_PULSES_PER_REVOLUTION = 2048.0;
    public static final double DRIVETRAIN_STEER_GEAR_RATIO = 18.0; // set correctly
    public static final double DRIVETRAIN_STEER_DEGREES = 360.0;
    public static final double DRIVETRAIN_STEER_PULSE_DISTANCE = HardwareConstants.DRIVETRAIN_STEER_DEGREES / (HardwareConstants.DRIVETRAIN_STEER_GEAR_RATIO * HardwareConstants.DRIVETRAIN_STEER_ENCODER_PULSES_PER_REVOLUTION);
    public static final double DRIVETRAIN_STEER_TICKS_PER_DEGREE = (HardwareConstants.DRIVETRAIN_STEER_GEAR_RATIO * HardwareConstants.DRIVETRAIN_STEER_ENCODER_PULSES_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_STEER_DEGREES;

    public static final double DRIVETRAIN_DRIVE_ENCODER_PULSES_PER_REVOLUTION = 2048.0;
    public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 8.95; // confirmed
    public static final double DRIVETRAIN_DRIVE_WHEEL_DIAMETER = 4.0; // (in inches)
    public static final double DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_DRIVE_PULSE_DISTANCE = HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / (HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.DRIVETRAIN_DRIVE_ENCODER_PULSES_PER_REVOLUTION);
    public static final double DRIVETRAIN_DRIVE_TICKS_PER_INCH = (HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.DRIVETRAIN_DRIVE_ENCODER_PULSES_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = 10.0 * HardwareConstants.DRIVETRAIN_DRIVE_PULSE_DISTANCE; // converts #ticks per 100ms into inches per second.
    public static final double DRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = 0.1 * HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into #ticks per 100ms.

    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 21.5; // (in inches)
    public static final double DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 25.5; // (in inches)
    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //================================================== Intake ==============================================================

    public static final boolean POWERCELL_ROLLER_MOTOR_INVERT_OUTPUT = false;
    public static final boolean POWERCELL_FLYWHEEL_MASTER_INVERT_OUTPUT = true;
    public static final boolean POWERCELL_FLYWHEEL_MASTER_INVERT_SENSOR = true;
    public static final boolean POWERCELL_FLYWHEEL_FOLLOWER_INVERT_OUTPUT = true;
    public static final boolean POWERCELL_CAROUSEL_MOTOR_INVERT_OUTPUT = true;
    public static final boolean POWERCELL_KICKER_MOTOR_INVERT_OUTPUT = false;

    //================================================== Climber ==============================================================

    public static final boolean CLIMBER_WINCH_MASTER_INVERT_OUTPUT = false;
    
    //================================================== Control Panel Spinner ==============================================================

    public static final boolean CONTROLPANEL_SPINNER_INVERT_OUTPUT = false;
}