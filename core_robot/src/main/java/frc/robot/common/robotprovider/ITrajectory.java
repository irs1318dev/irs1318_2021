package frc.robot.common.robotprovider;

public interface ITrajectory
{
    TrajectoryState get(double time);
    double getDuration();
}