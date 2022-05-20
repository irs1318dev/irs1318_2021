package frc.robot.common.robotprovider;

import com.google.inject.Singleton;

import edu.wpi.first.wpilibj.DriverStation;

@Singleton
public class DriverStationWrapper implements IDriverStation
{
    public DriverStationWrapper()
    {
    }

    @Override
    public String getEventName()
    {
        return DriverStation.getEventName();
    }

    @Override
    public Alliance getAlliance()
    {
        switch (DriverStation.getAlliance())
        {
            case Red:
                return Alliance.Red;

            case Blue:
                return Alliance.Blue;

            default:
            case Invalid:
                return Alliance.Invalid;
        }
    }

    @Override
    public int getLocation()
    {
        return DriverStation.getLocation();
    }

    @Override
    public int getMatchNumber()
    {
        return DriverStation.getMatchNumber();
    }

    @Override
    public MatchType getMatchType()
    {
        switch (DriverStation.getMatchType())
        {
            case Practice:
                return MatchType.Practice;

            case Qualification:
                return MatchType.Qualification;

            case Elimination:
                return MatchType.Elimination;

            default:
            case None:
                return MatchType.None;
        }
    }

    @Override
    public int getReplayNumber()
    {
        return DriverStation.getReplayNumber();
    }

    @Override
    public double getMatchTime()
    {
        return DriverStation.getMatchTime();
    }

    @Override
    public RobotMode getMode()
    {
        if (!DriverStation.isEnabled())
        {
            return RobotMode.Disabled;
        }
        else if (DriverStation.isAutonomous())
        {
            return RobotMode.Autonomous;
        }
        else if (DriverStation.isTest())
        {
            return RobotMode.Test;
        }
        else
        {
            return RobotMode.Teleop;
        }
    }

    @Override
    public String getGameSpecificMessage()
    {
        return DriverStation.getGameSpecificMessage();
    }
}