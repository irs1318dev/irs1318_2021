package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.Pose2d;
import frc.robot.common.robotprovider.TrajectoryState;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.PathManager;
import frc.robot.mechanisms.DriveTrainMechanism;

/**
 * Task that follows a path
 * 
 */
public class FollowPathTask extends ControlTaskBase
{
    private final String pathName;
    private final boolean fromCurrentPose;

    private ITimer timer;

    private double startTime;
    private double duration;
    private ITrajectory trajectory;
    private Pose2d initialPose;

    /**
     * Initializes a new FollowPathTask
     */
    public FollowPathTask(String pathName)
    {
        this(pathName, true);
    }

    /**
     * Initializes a new FollowPathTask
     */
    public FollowPathTask(String pathName, boolean fromCurrentPose)
    {
        this.pathName = pathName;
        this.fromCurrentPose = fromCurrentPose;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        PathManager pathManager = this.getInjector().getInstance(PathManager.class);
        this.trajectory = pathManager.getTrajectory(this.pathName);

        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = this.timer.get();
        this.duration = this.trajectory.getDuration();

        if (this.fromCurrentPose)
        {
            DriveTrainMechanism driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);
            this.initialPose = driveTrain.getPose();
        }
        else
        {
            this.initialPose = new Pose2d(0.0, 0.0, 0.0);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        TrajectoryState state = this.trajectory.get(this.timer.get());
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXGoal, state.pose.x + this.initialPose.x);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYGoal, state.pose.y + this.initialPose.y);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleGoal, state.pose.angle + this.initialPose.angle);
        this.setAnalogOperationState(AnalogOperation.DriveTrainVelocityGoal, state.velocity);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainVelocityGoal, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.timer.get() > this.startTime + this.duration;
    }
}
