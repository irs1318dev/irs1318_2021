package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;

public class FlywheelFixedSpinTask extends TimedTask
{
    private final double speed;


    /**
     * Initializes a new FlywheelFixedSpinTask
     * @param speed to spin the flywheel
     */
    public FlywheelFixedSpinTask(double speed)
    {
        this(speed, 10.0);
    }

    /**
     * Initializes a new FlywheelFixedSpinTask
     * @param speed to spin the flywheel
     */
    public FlywheelFixedSpinTask(double speed, double duration)
    {
        super(duration);

        this.speed = speed;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setAnalogOperationState(AnalogOperation.PowerCellFlywheelVelocity, this.speed);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.PowerCellFlywheelVelocity, this.speed);
    }
}
