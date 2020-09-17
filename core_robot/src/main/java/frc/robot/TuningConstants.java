package frc.robot;

import java.io.IOException;
import java.util.*;

import com.google.inject.Injector;

import frc.robot.common.*;
import frc.robot.common.robotprovider.Alliance;
import frc.robot.common.robotprovider.CSVLogger;
import frc.robot.common.robotprovider.IDriverStation;
import frc.robot.common.robotprovider.IFile;
import frc.robot.common.robotprovider.ILogger;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.ISmartDashboardLogger;
import frc.robot.common.robotprovider.MatchType;
import frc.robot.common.robotprovider.MultiLogger;
import frc.robot.mechanisms.*;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;

    public static final int CALENDAR_YEAR = 2020;
    public static final boolean LOG_TO_FILE = true; //TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false; // true;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    public static final double MAGIC_NULL_VALUE = -1318.0;

    public static List<IMechanism> GetActiveMechanisms(Injector injector)
    {
        List<IMechanism> mechanismList = new ArrayList<IMechanism>();
        // mechanismList.add(injector.getInstance(DriveTrainMechanism.class));
        // mechanismList.add(injector.getInstance(PowerManager.class));
        // mechanismList.add(injector.getInstance(PositionManager.class));
        // mechanismList.add(injector.getInstance(CompressorMechanism.class));
        // mechanismList.add(injector.getInstance(OffboardVisionManager.class));
        // mechanismList.add(injector.getInstance(IndicatorLightManager.class));
        return mechanismList;
    }

    public static ILogger GetLogger(Injector injector)
    {
        ISmartDashboardLogger smartDashboardLogger = injector.getInstance(ISmartDashboardLogger.class);
        if (!TuningConstants.LOG_TO_FILE)
        {
            return smartDashboardLogger;
        }

        IRobotProvider robotProvider = injector.getInstance(IRobotProvider.class);
        IDriverStation driverStation = robotProvider.getDriverStation();
        MatchType matchType = driverStation.getMatchType();
        if (matchType == MatchType.None && TuningConstants.LOG_FILE_ONLY_COMPETITION_MATCHES)
        {
            return smartDashboardLogger;
        }

        // NI's Linux RTOS automatically mounts USB Sticks as /U/ and /V/, so long as they are formatted as FAT32
        // https://knowledge.ni.com/KnowledgeArticleDetails?id=kA00Z000000P7iaSAC&l=en-US
        // https://www.ni.com/en-us/support/documentation/supplemental/18/file-system-compatibility-with-the-ni-linux-real-time-os.html
        IFile rootDirectory = injector.getInstance(IFile.class);
        rootDirectory.open("/U/");
        if (!rootDirectory.exists() || rootDirectory.getFreeSpace() < TuningConstants.LOG_FILE_REQUIRED_FREE_SPACE)
        {
            return smartDashboardLogger;
        }

        String eventName = driverStation.getEventName();
        int matchNumber = driverStation.getMatchNumber();
        int replayNumber = driverStation.getReplayNumber();
        Alliance alliance = driverStation.getAlliance();
        int location = driverStation.getLocation();
        IFile file;
        if (eventName == null ||
            matchType == MatchType.None ||
            matchNumber == 0 ||
            alliance == Alliance.Invalid ||
            location <= 0 ||
            location >= 4)
        {
            if (TuningConstants.LOG_FILE_ONLY_COMPETITION_MATCHES)
            {
                // strange 
                return smartDashboardLogger;
            }
            else
            {
                // don't know important information, let's just put it under the root of the USB stick
                IFile directory = injector.getInstance(IFile.class);
                directory.open("/U/other/");
                directory.mkdir();

                file = injector.getInstance(IFile.class);
                file.open(String.format("/U/other/%1$d.csv", Calendar.getInstance().getTime().getTime()));
                if (file.exists())
                {
                    // file already exists
                    return smartDashboardLogger;
                }
            }
        }
        else
        {
            String directoryPath = String.format("/U/%1$d - %2$s/", TuningConstants.CALENDAR_YEAR, eventName);
            IFile directory = injector.getInstance(IFile.class);
            directory.open(directoryPath);
            directory.mkdir();

            // name the file a la "/U/2020 - Glacier Peak/Q03 (R2).auto.csv" or "/U/2020 - Glacier Peak/Q12R1 (B3).tele.csv"
            boolean isAuto = driverStation.isAutonomous();
            file = injector.getInstance(IFile.class);
            String fileName =
                String.format(
                    "%1$s%2$s%3$02d%4$s (%5$s%6$d).%7$s.csv",
                    directoryPath,
                    matchType.value,
                    matchNumber,
                    replayNumber == 0 ? "" : String.format("R%1$d", replayNumber),
                    alliance.value,
                    location,
                    isAuto ? "auto" : "tele");

            file.open(fileName);
            if (file.exists())
            {
                for (int i = 2; i <= 5; i++)
                {
                    // start adding .2, .3, etc. to the name, a la "/U/2020 - Glacier Peak/Q03 (R2).2.csv"
                    fileName =
                        String.format(
                            "%1$s%2$s%3$02d%4$s (%5$s%6$d).%7$s.%8$d.csv",
                            directoryPath,
                            matchType.value,
                            matchNumber,
                            replayNumber == 0 ? "" : String.format("R%1$d", replayNumber),
                            alliance.value,
                            location,
                            isAuto ? "auto" : "tele",
                            i);

                    file.open(fileName);
                    if (!file.exists())
                    {
                        break;
                    }
                }
            }
        }

        try
        {
            CSVLogger csvLogger = new CSVLogger(file.openWriter());
            return new MultiLogger(csvLogger, smartDashboardLogger);
        }
        catch (IOException ex)
        {
            return smartDashboardLogger;
        }
    }

    //================================================== Autonomous ==============================================================

    public static final boolean CANCEL_AUTONOMOUS_ROUTINE_ON_DISABLE = true;

    public static final double DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA = 1.0;

    // Navx Turn Constants
    public static final double MAX_NAVX_TURN_RANGE_DEGREES = 5.0;
    public static final double MAX_NAVX_FAST_TURN_RANGE_DEGREES = 5.0;
    public static final double NAVX_FAST_TURN_TIMEOUT = 1.25;
    public static final double NAVX_TURN_COMPLETE_TIME = 0.4;
    public static final double NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA = 0;
    public static final double NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA = 0;

    // Navx Turn PID Constants
    public static final double NAVX_TURN_PID_KP = 0.025; // 0.04
    public static final double NAVX_TURN_PID_KI = 0.0;
    public static final double NAVX_TURN_PID_KD = 0.02;
    public static final double NAVX_TURN_PID_KF = 0.0;
    public static final double NAVX_TURN_PID_KS = 1.0;
    public static final double NAVX_TURN_PID_MIN = -0.8;
    public static final double NAVX_TURN_PID_MAX = 0.8;
    public static final double NAVX_FAST_TURN_PID_KP = 0.01;
    public static final double NAVX_FAST_TURN_PID_KI = 0.0;
    public static final double NAVX_FAST_TURN_PID_KD = 0.0;
    public static final double NAVX_FAST_TURN_PID_KF = 0.0;
    public static final double NAVX_FAST_TURN_PID_KS = 1.0;
    public static final double NAVX_FAST_TURN_PID_MIN = -0.8;
    public static final double NAVX_FAST_TURN_PID_MAX = 0.8;

    // Kinodynamic constraints for driving with roadrunner
    public static final double ROADRUNNER_TIME_STEP = 0.01;
    public static final double ROADRUNNER_MAX_VELOCITY = 100.0;
    public static final double ROADRUNNER_MAX_ACCELERATION = 200.0;
    public static final double ROADRUNNER_MAX_JERK = 2000.0;
    public static final double ROADRUNNER_MAX_ANGULAR_VELOCITY = 5.0;
    public static final double ROADRUNNER_MAX_ANGULAR_ACCELERATION = 10.0;
    public static final double ROADRUNNER_MAX_ANGULAR_JERK = 20.0;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;
    public static final double MAX_VISION_TURRET_CENTERING_RANGE_DEGREES = 1.0;

    // Acceptable vision distance from tape in inches
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 3.25;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.02;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.02;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.3;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.3;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.02;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.03;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.01;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static final double VISION_FAST_ADVANCING_PID_KP = 0.01;
    public static final double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static final double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static final double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static final boolean VISION_ENABLE_DURING_TELEOP = true;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_USE_PID = true;

    public static final double ANGLE_MOTOR_POSITION_PID_KP = 0.0;
    public static final double ANGLE_MOTOR_POSITION_PID_KI = 0.0;
    public static final double ANGLE_MOTOR_POSITION_PID_KD = 0.0;
    public static final double ANGLE_MOTOR_POSITION_PID_KF = 0.0;
    public static final double ANGLE_MOTOR_VELOCITY_PID_KS = 0.0; 

    // Velocity PID (drive)
    public static final double DRIVE_MOTOR_VELOCITY_PID_KP = 0.0;
    public static final double DRIVE_MOTOR_VELOCITY_PID_KI = 0.0;
    public static final double DRIVE_MOTOR_VELOCITY_PID_KD = 0.0;
    public static final double DRIVE_MOTOR_VELOCITY_PID_KF = 0.0; 
    public static final double DRIVE_MOTOR_VELOCITY_PID_KS = 0.0; 

    public static final double DRIVETRAIN_K1 = 0.0;
    public static final double DRIVETRAIN_K2 = 0.0;

    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 0.0;

    public static final double DRIVETRAIN_DEAD_ZONE = 0.05;


    
}
