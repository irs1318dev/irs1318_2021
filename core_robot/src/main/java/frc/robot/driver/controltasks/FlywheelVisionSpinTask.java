package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that changes the flywheel speed based based on vision center (distance to target)
 */
public class FlywheelVisionSpinTask extends ControlTaskBase
{
    private static final int NO_CENTER_THRESHOLD = 40;
    private static final double[][] HOOD_RANGES =
        {
            {120, 200}, // short
            {200, 400}, // medium
            {400, 600}, // long
        };

    private OffboardVisionManager visionManager;

    private Double distance;
    private int noCenterCount;

    /**
    * Initializes a new FlyWheelVelocityTask
    */
    public FlywheelVisionSpinTask()
    {
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.distance = this.visionManager.getDistance();

        if (this.distance != null)
        {
            double flywheelSpeed = 0.0;
            if (this.distance >= FlywheelVisionSpinTask.HOOD_RANGES[0][0] &&
                this.distance < FlywheelVisionSpinTask.HOOD_RANGES[0][1])
            {
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodPointBlank, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodShort, true);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodMedium, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodLong, false);
                flywheelSpeed = 0.0; // do calculations to find speed based off distance
            }
            else if (this.distance >= FlywheelVisionSpinTask.HOOD_RANGES[1][0] &&
                this.distance < FlywheelVisionSpinTask.HOOD_RANGES[1][1])
            {
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodPointBlank, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodShort, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodMedium, true);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodLong, false);;
                flywheelSpeed = 0.0; // do calculations to find speed based off distance
            }
            else if (this.distance >= FlywheelVisionSpinTask.HOOD_RANGES[2][0] &&
                this.distance < FlywheelVisionSpinTask.HOOD_RANGES[2][1])
            {
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodPointBlank, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodShort, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodMedium, false);
                this.setDigitalOperationState(DigitalOperation.PowerCellHoodLong, true);
                flywheelSpeed = 0.0; // do calculations to find speed based off distance
            }

            this.setAnalogOperationState(AnalogOperation.PowerCellFlywheelVelocity, flywheelSpeed);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);

        this.setDigitalOperationState(DigitalOperation.PowerCellHoodPointBlank, false);
        this.setDigitalOperationState(DigitalOperation.PowerCellHoodShort, false);
        this.setDigitalOperationState(DigitalOperation.PowerCellHoodMedium, false);
        this.setDigitalOperationState(DigitalOperation.PowerCellHoodLong, false);
        this.setAnalogOperationState(AnalogOperation.PowerCellFlywheelVelocity, TuningConstants.PERRY_THE_PLATYPUS);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return false;
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

        return this.noCenterCount >= FlywheelVisionSpinTask.NO_CENTER_THRESHOLD;
    }
}