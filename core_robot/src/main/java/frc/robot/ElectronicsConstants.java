package frc.robot;

import frc.robot.common.robotprovider.PneumaticsModuleType;
import frc.robot.common.robotprovider.PowerDistributionModuleType;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // We expect the following to be true.  Change INVERT_*_AXIS to true if any of the following are not met:
    // 1. forwards/up on a joystick is positive, backwards/down is negative.
    // 2. right on a joystick is positive, left on a joystick is negative.
    // 3. pressed on a trigger is positive, released is negative/zero.
    public static final boolean INVERT_XBONE_LEFT_X_AXIS = false;
    public static final boolean INVERT_XBONE_RIGHT_X_AXIS = false;
    public static final boolean INVERT_XBONE_LEFT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_LEFT_TRIGGER = false;
    public static final boolean INVERT_XBONE_RIGHT_TRIGGER = false;

    public static final boolean INVERT_PS4_LEFT_X_AXIS = false;
    public static final boolean INVERT_PS4_RIGHT_X_AXIS = false;
    public static final boolean INVERT_PS4_LEFT_Y_AXIS = true;
    public static final boolean INVERT_PS4_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_PS4_LEFT_TRIGGER = false;
    public static final boolean INVERT_PS4_RIGHT_TRIGGER = false;

    public static final boolean INVERT_THROTTLE_AXIS = true;
    public static final boolean INVERT_TRIGGER_AXIS = false;

    public static final int POWER_DISTRIBUTION_CAN_ID = 0;
    public static final PowerDistributionModuleType POWER_DISTRIBUTION_TYPE = PowerDistributionModuleType.PowerDistributionPanel;

    public static final String CANIVORE_NAME = "CANIVORE1"; // Module A

    public static final int PNEUMATICS_MODULE_A = 0; // Module A
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_A = PneumaticsModuleType.PneumaticsControlModule; // Module A
    public static final int PNEUMATICS_MODULE_B = 1; // Module B
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_B = PneumaticsModuleType.PneumaticsControlModule; // Module B

    public static final boolean PNEUMATICS_USE_HYBRID = false;
    public static final boolean PNEUMATICS_USE_ANALOG = false;
    public static final double PNEUMATICS_MIN_PSI = 110.0;
    public static final double PNEUMATICS_MAX_PSI = 120.0;

    //================================================== IMU ==============================================================

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
