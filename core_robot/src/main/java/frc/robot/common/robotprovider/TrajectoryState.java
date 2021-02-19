package frc.robot.common.robotprovider;

public class TrajectoryState
{
    // The time elapsed since the beginning of the trajectory, in seconds
    public double time;

    // The speed at that point of the trajectory, in inches per second
    public double velocity;

    // The acceleration at that point of the trajectory, in inches per second squared
    public double acceleration;

    // The pose at that point of the trajectory, in inches
    public Pose2d pose;

    // The curvature at that point of the trajectory, in degrees per inch
    public double curvature;

    public TrajectoryState()
    {
    }

    /**
     * Initializes a new instance of the TrajectoryState class
     * @param time elapsed in seconds
     * @param velocity in inches per second
     * @param acceleration in inches per second squared
     * @param pose in inches and degrees
     * @param curvature in degrees per inch
     */
    public TrajectoryState(
        double time,
        double velocity,
        double acceleration,
        Pose2d pose,
        double curvature)
    {
        this.time = time;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.pose = pose;
        this.curvature = curvature;
    }
}