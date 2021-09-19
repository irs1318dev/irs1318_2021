package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionCenteringTask extends ControlTaskBase
{
    private static final int NO_CENTER_THRESHOLD = 40;

    private final boolean useTime;

    protected OffboardVisionManager visionManager;
    private ITimer timer;
    private PIDHandler turnPidHandler;

    private Double centeredTime;

    private int noCenterCount;

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionCenteringTask()
    {
        this(true);
    }

    /**
    * Initializes a new VisionCenteringTask
    * @param useTime whether to make sure we are centered for a second or not
    */
    public VisionCenteringTask(boolean useTime)
    {
        this.useTime = useTime;

        this.turnPidHandler = null;
        this.centeredTime = null;

        this.noCenterCount = 0;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.turnPidHandler = this.createTurnHandler();

        if (this.useTime)
        {
            this.timer = this.getInjector().getInstance(ITimer.class);
        }

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        Double currentMeasuredAngle = this.visionManager.getHorizontalAngle();
        if (currentMeasuredAngle != null)
        {
            this.setAnalogOperationState(
                AnalogOperation.DriveTrainTurnSpeed,
                this.turnPidHandler.calculatePosition(0.0, currentMeasuredAngle));
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnSpeed, 0.0);

        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        Double currentMeasuredAngle = this.visionManager.getHorizontalAngle();
        if (currentMeasuredAngle == null)
        {
            return false;
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle);
        if (centerAngleDifference > TuningConstants.MAX_VISION_CENTERING_RANGE_DEGREES)
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }

        // otherwise, use time:
        if (this.centeredTime == null)
        {
            this.centeredTime = this.timer.get();
            return false;
        }
        else if (this.timer.get() - this.centeredTime < 0.75)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * Checks whether this task should be stopped, or whether it should continue being processed.
     * @return true if we should cancel this task (and stop performing any subsequent tasks), otherwise false (to keep processing this task)
     */
    @Override
    public boolean shouldCancel()
    {
        if (this.visionManager.getDistance() == null)
        {
            this.noCenterCount++;
        }
        else
        {
            this.noCenterCount = 0;
        }

        return this.noCenterCount >= VisionCenteringTask.NO_CENTER_THRESHOLD;
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KP,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KI,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KD,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KF,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KS,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_MIN,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}