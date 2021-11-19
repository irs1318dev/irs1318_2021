package frc.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_XBONE_LEFT_X_AXIS = false;
    public static final boolean INVERT_XBONE_RIGHT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_XBONE_LEFT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_RIGHT_Y_AXIS = true;

    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_PS4_LEFT_X_AXIS = false;
    public static final boolean INVERT_PS4_RIGHT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_PS4_LEFT_Y_AXIS = true;
    public static final boolean INVERT_PS4_RIGHT_Y_AXIS = true;

    // change INVERT_THROTTLE_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_THROTTLE_AXIS = true;

    // change INVERT_TRIGGER_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_TRIGGER_AXIS = false;

    public static final int PCM_A_MODULE = 0; // Module A
    public static final int PCM_B_MODULE = 1; // Module B

    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    //================================================== Vision ==============================================================

    public static final int PIGEON_IMU_CAN_ID = 42;

    //================================================== Vision ==============================================================

    public static final int VISION_RING_LIGHT_DIO = 0;

    //================================================== Indicator Lights ==============================================================

    public static final int INDICATOR_LIGHT_X_DIO = -1;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_STEER_MOTOR_1_CAN_ID = 1;
    public static final int DRIVETRAIN_DRIVE_MOTOR_1_CAN_ID = 2;
    public static final int DRIVETRAIN_STEER_MOTOR_2_CAN_ID = 3;
    public static final int DRIVETRAIN_DRIVE_MOTOR_2_CAN_ID = 4;
    public static final int DRIVETRAIN_STEER_MOTOR_3_CAN_ID = 5;
    public static final int DRIVETRAIN_DRIVE_MOTOR_3_CAN_ID = 6;
    public static final int DRIVETRAIN_STEER_MOTOR_4_CAN_ID = 7;
    public static final int DRIVETRAIN_DRIVE_MOTOR_4_CAN_ID = 8;

    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_1_ANALOG_INPUT = 0;
    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_2_ANALOG_INPUT = 1;
    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_3_ANALOG_INPUT = 2;
    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_4_ANALOG_INPUT = 3;

    //================================================== Intake ==============================================================

    public static final int POWERCELL_INTAKE_FORWARD_PCM = 3;
    public static final int POWERCELL_INTAKE_REVERSE_PCM = 2;
    public static final int POWERCELL_KICKER_FORWARD_PCM = 1;
    public static final int POWERCELL_KICKER_REVERSE_PCM = 0;
    public static final int POWERCELL_OUTER_HOOD_FORWARD_PCM = 4;
    public static final int POWERCELL_OUTER_HOOD_REVERSE_PCM = 5;
    public static final int POWERCELL_INNER_HOOD_FORWARD_PCM = 6;
    public static final int POWERCELL_INNER_HOOD_REVERSE_PCM = 7;

    public static final int POWERCELL_ROLLER_MOTOR_CAN_ID = 9;
    public static final int POWERCELL_FLYWHEEL_MASTER_CAN_ID = 10;
    public static final int POWERCELL_FLYWHEEL_FOLLOWER_CAN_ID = 11;
    public static final int POWERCELL_CAROUSEL_MOTOR_CAN_ID = 12;
    public static final int POWERCELL_KICKER_MOTOR_CAN_ID = 13;

    public static final int POWERCELL_CAROUSEL_COUNTER_DIO = 0;
    public static final int POWERCELL_THROUGHBEAM_ANALOG_INPUT = 0;
    public static final int POWERCELL_CAROUSEL_ENCODER_CHANNEL_A = 2;
    public static final int POWERCELL_CAROUSEL_ENCODER_CHANNEL_B = 3;

    //================================================== Climber ==============================================================

    public static final int CLIMBER_EXTEND_FORWARD_PCM = 42;
    public static final int CLIMBER_EXTEND_REVERSE_PCM = 43;
    public static final int CLIMBER_GRAB_FORWARD_PCM = 44;
    public static final int CLIMBER_GRAB_REVERSE_PCM = 45;

    public static final int CLIMBER_WINCH_MASTER_CAN_ID = 14;

    //================================================== Control Panel Spinner ==============================================================

    public static final int CONTROLPANEL_SPINNER_CAN_ID = 15;
    public static final int CONTROLPANEL_EXTENDER_FORWARD_PCM = 16;
    public static final int CONTROLPANEL_EXTENDER_REVERSE_PCM = 17;
}
