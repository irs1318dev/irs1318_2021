package frc.robot.common.robotprovider;

public interface ITrajectoryGenerator
{
    ITrajectory generateTrajectory(Pose2d start, Pose2d end, Point2d[] translations);
}