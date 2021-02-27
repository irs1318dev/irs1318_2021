package frc.robot.common.robotprovider;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.common.Helpers;

public class TrajectoryWrapper implements ITrajectory
{
    private final Trajectory wrappedObject;

    public TrajectoryWrapper(Trajectory object)
    {
        this.wrappedObject = object;
    }

    public TrajectoryState get(double time)
    {
        State state = this.wrappedObject.sample(time);
        if (state == null)
        {
            return null;
        }

        return new TrajectoryState(
            state.timeSeconds,
            state.velocityMetersPerSecond * Helpers.INCHES_PER_METER,
            state.accelerationMetersPerSecondSq * Helpers.INCHES_PER_METER,
            new Pose2d(state.poseMeters.getX() * Helpers.INCHES_PER_METER,
                state.poseMeters.getY() * Helpers.INCHES_PER_METER,
                state.poseMeters.getRotation().getRadians() * Helpers.RADIANS_TO_DEGREES),
            (state.curvatureRadPerMeter * Helpers.RADIANS_TO_DEGREES) / Helpers.INCHES_PER_METER);
    }

    public double getDuration()
    {
        return this.wrappedObject.getTotalTimeSeconds();
    }
}
