package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;

    //================================================== Magic Values ==============================================================

    public static double MAGIC_NULL_VALUE = -1318.0;
    public static double PERRY_THE_PLATYPUS = 0.0;

    //================================================== Logging  ==============================================================

    public static int CALENDAR_YEAR = 2021;
    public static boolean LOG_TO_FILE = TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_FILE_ONLY_COMPETITION_MATCHES = true;
    public static long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static int LOG_FLUSH_THRESHOLD = 25;

    //================================================== Autonomous ==============================================================

    public static boolean CANCEL_AUTONOMOUS_ROUTINE_ON_DISABLE = true;

    public static double DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA = 1.0;

    // Navx Turn Constants
    public static double MAX_NAVX_TURN_RANGE_DEGREES = 5.0;
    public static double MAX_NAVX_FAST_TURN_RANGE_DEGREES = 5.0;
    public static double NAVX_FAST_TURN_TIMEOUT = 1.25;
    public static double NAVX_TURN_COMPLETE_TIME = 0.4;
    public static double NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA = 0;
    public static double NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA = 0;

    // Navx Turn PID Constants
    public static double NAVX_TURN_PID_KP = 0.025; // 0.04
    public static double NAVX_TURN_PID_KI = 0.0;
    public static double NAVX_TURN_PID_KD = 0.02;
    public static double NAVX_TURN_PID_KF = 0.0;
    public static double NAVX_TURN_PID_KS = 1.0;
    public static double NAVX_TURN_PID_MIN = -0.8;
    public static double NAVX_TURN_PID_MAX = 0.8;
    public static double NAVX_FAST_TURN_PID_KP = 0.01;
    public static double NAVX_FAST_TURN_PID_KI = 0.0;
    public static double NAVX_FAST_TURN_PID_KD = 0.0;
    public static double NAVX_FAST_TURN_PID_KF = 0.0;
    public static double NAVX_FAST_TURN_PID_KS = 1.0;
    public static double NAVX_FAST_TURN_PID_MIN = -0.8;
    public static double NAVX_FAST_TURN_PID_MAX = 0.8;

    // Kinodynamic constraints for driving with roadrunner
    public static double ROADRUNNER_TIME_STEP = 0.01;
    public static double ROADRUNNER_MAX_VELOCITY = 100.0;
    public static double ROADRUNNER_MAX_ACCELERATION = 200.0;
    public static double ROADRUNNER_MAX_JERK = 2000.0;
    public static double ROADRUNNER_MAX_ANGULAR_VELOCITY = 5.0;
    public static double ROADRUNNER_MAX_ANGULAR_ACCELERATION = 10.0;
    public static double ROADRUNNER_MAX_ANGULAR_JERK = 20.0;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;
    public static double MAX_VISION_TURRET_CENTERING_RANGE_DEGREES = 1.0;

    // Acceptable vision distance from tape in inches
    public static double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 3.25;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static double VISION_STATIONARY_CENTERING_PID_KP = 0.0125;
    public static double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static double VISION_STATIONARY_CENTERING_PID_KD = 0.01;
    public static double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static double VISION_STATIONARY_CENTERING_PID_MIN = -0.4;
    public static double VISION_STATIONARY_CENTERING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target
    public static double VISION_MOVING_CENTERING_PID_KP = 0.02;
    public static double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static double VISION_MOVING_CENTERING_PID_KD = 0.03;
    public static double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static double VISION_ADVANCING_PID_KP = 0.01;
    public static double VISION_ADVANCING_PID_KI = 0.0;
    public static double VISION_ADVANCING_PID_KD = 0.0;
    public static double VISION_ADVANCING_PID_KF = 0.0;
    public static double VISION_ADVANCING_PID_KS = 1.0;
    public static double VISION_ADVANCING_PID_MIN = -0.3;
    public static double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static double VISION_FAST_ADVANCING_PID_KP = 0.01;
    public static double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static boolean VISION_ENABLE_DURING_TELEOP = true;
    public static int VISION_MISSED_HEARTBEAT_THRESHOLD = 10000;

    //================================================== Indicator Lights ========================================================

    public static double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    //================================================== DriveTrain ==============================================================

    public static boolean DRIVETRAIN_USE_PID = true;
    public static boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static boolean DRIVETRAIN_RESET_ON_ROBOT_START = false;

    //public static double[] DRIVETRAIN_STEER_MOTOR_ABSOLUTE_OFFSET = new double[] { -135.0, 65.0, -96.0, -14.0 };
    public static double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = -136.6;
    public static double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = 65.2;
    public static double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = -94.1;
    public static double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -37.1;

    // Position PID (angle) per-module
    public static double DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KF = 0.0;

    // Velocity PID (drive) per-module
    public static double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks. this is #ticks / 100ms

    public static double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KP = 0.09;
    public static double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KP = 0.09;
    public static double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KP = 0.09;
    public static double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KP = 0.09;
    public static double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KF = 0.0;

    public static double DRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static double DRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static double DRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static double DRIVETRAIN_OMEGA_MAX_OUTPUT = TuningConstants.DRIVETRAIN_TURN_SCALE;
    public static double DRIVETRAIN_OMEGA_MIN_OUTPUT = -TuningConstants.DRIVETRAIN_TURN_SCALE;

    public static double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static double DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = TuningConstants.DRIVETRAIN_TURN_SCALE;
    public static double DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -TuningConstants.DRIVETRAIN_TURN_SCALE;

    public static double DRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.0;
    public static double DRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static double DRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static double DRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static double DRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static double DRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static double DRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static boolean DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static double DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 11.0;
    public static boolean DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static double DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 30.0;
    public static double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 100.0;

    public static boolean DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static double DRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 11.0;
    public static boolean DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static double DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 30.0;
    public static double DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static double DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 100.0;

    public static boolean DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static double DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.25;

    public static double DRIVETRAIN_DEAD_ZONE_TURN = 0.20;
    public static double DRIVETRAIN_DEAD_ZONE_VELOCITY = 0.15;
    public static double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.15;

    public static double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static double DRIVETRAIN_TURN_SCALE = 4.0; // radians per second
    public static double DRIVETRAIN_TURN_APPROXIMATION = 0.5; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 45.0; // in degrees per second
    public static double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.80 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second

    //================================================== Intake ==============================================================

    public static double POWERCELL_ROLLER_MOTOR_INTAKE_POWER = 0.5;
    public static double POWERCELL_ROLLER_MOTOR_OUTTAKE_POWER = -0.5;

    public static double POWERCELL_KICKER_MOTOR_FEED_POWER = 0.35;
    public static double POWERCELL_KICKER_MOTOR_FEED_REVERSE_POWER = -0.5;

    public static double POWERCELL_FLYWHEEL_REVERSE_POWER = -0.4;

    public static double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KP = 0.55;
    public static double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KI = 0.0;
    public static double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KD = 0.75;
    public static double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KF = 0.0113666; // 90000 / 1023
    public static double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KS = 85000.0;

    public static int POWERCELL_FLYWHEEL_VELOCITY_PERIOD = 10;
    public static int POWERCELL_FLYWHEEL_VELOCITY_WINDOWSIZE = 32;
    public static boolean POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED = true;
    public static boolean POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED = true;
    public static double POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE = 12.0;
    public static double POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE = 12.0;

    public static double POWERCELL_THROUGHBEAM_CUTOFF = 2.7;
    public static double POWERCELL_CAROUSEL_COUNT_THRESHOLD = 5.0;
    public static double POWERCELL_CAROUSEL_MOTOR_POWER_INDEXING = -0.4;
    public static double POWERCELL_CAROUSEL_MOTOR_POWER_SHOOTING = 0.4;
    public static double POWERCELL_CAROUSEL_MECHANISM_INDEXING_TIMEOUT = 2.0;
    public static double POWERCELL_FLYWHEEL_ALLOWABLE_ERROR_RANGE = 500; // ticks per 100ms

    public static boolean POWERCELL_CAROUSEL_USE_PID = true;

    public static double POWERCELL_CAROUSEL_PID_KP = 0.0002;
    public static double POWERCELL_CAROUSEL_PID_KI = 0.0;
    public static double POWERCELL_CAROUSEL_PID_KD = 0.0;
    public static double POWERCELL_CAROUSEL_PID_KF = 1.0;
    public static double POWERCELL_CAROUSEL_PID_KS = 3200.0;
    public static double POWERCELL_CAROUSEL_MAX_POWER = 1.0;
}
