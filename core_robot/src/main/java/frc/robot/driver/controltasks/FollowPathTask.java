package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.Pose2d;
import frc.robot.common.robotprovider.TrajectoryState;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.PathManager;
import frc.robot.mechanisms.DriveTrainMechanism;

/**
 * Task that follows a path
 * 
 */
public class FollowPathTask extends ControlTaskBase
{
    private final String pathName;
    private final boolean fromCurrentPose;
    private final boolean maintainInitialAngle;

    private ITimer timer;

    private double startTime;
    private double trajectoryDuration;
    private ITrajectory trajectory;
    private Pose2d initialPose;

    /**
     * Initializes a new FollowPathTask
     */
    public FollowPathTask(String pathName)
    {
        this(pathName, true, true);
    }

    /**
     * Initializes a new FollowPathTask
     */
    public FollowPathTask(String pathName, boolean fromCurrentPose, boolean maintainInitialAngle)
    {
        this.pathName = pathName;
        this.fromCurrentPose = fromCurrentPose;
        this.maintainInitialAngle = maintainInitialAngle;
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
        this.trajectoryDuration = this.trajectory.getDuration();

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
        TrajectoryState state = this.trajectory.get(this.timer.get() - this.startTime);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXGoal, state.xPosition + this.initialPose.x);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYGoal, state.yPosition + this.initialPose.y);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, state.angle);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXVelocityGoal, state.xVelocity);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYVelocityGoal, state.yVelocity);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleVelocityGoal, state.angleVelocity);
        if (this.maintainInitialAngle)
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleReference, this.initialPose.angle);
        }
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
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXVelocityGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYVelocityGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleVelocityGoal, 0.0);
        if (this.maintainInitialAngle)
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleReference, 0.0);
        }
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.timer.get() > this.startTime + this.trajectoryDuration;
    }
}
