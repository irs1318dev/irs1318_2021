package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.driver.PathManager;

/**
 * Task that follows a path
 * 
 */
public class FollowPathTask extends ControlTaskBase
{
    private final String pathName;

    private ITimer timer;

    private double startTime;
    private double duration;
    private ITrajectory trajectory;

    /**
     * Initializes a new FollowPathTask
     */
    public FollowPathTask(String pathName)
    {
        this.pathName = pathName;
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
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
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
